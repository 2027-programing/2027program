package first.robot.mechanisms;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.system.Notifier;
import org.wpilib.system.RobotController;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.alerts.AlertableCollection;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import first.robot.constants.DriverConstants;
import first.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Swerve 底盤（Commands v3 的 Mechanism）。
 *
 * <p>對應 2026 的 {@code CommandSwerveDrivetrain}。差別：
 *
 * <ul>
 *   <li>2026：{@code extends TunerSwerveDrivetrain implements Subsystem}（Commands v2）
 *   <li>2027：{@code extends TunerSwerveDrivetrain implements Mechanism}（Commands v3）
 * </ul>
 *
 * <p>底層的 swerve 控制（模組角度、速度、里程計）一樣交給 CTRE Phoenix 6 的 SwerveDrivetrain， 這個類別只負責把它包成
 * v3 的指令（command）。
 *
 * <p>v3 的 Mechanism 沒有 {@code periodic()}，所以每圈要做的事放在 {@link #updateEachLoop()}，由 Robot 呼叫。
 */
public class SwerveDrive extends TunerSwerveDrivetrain implements Mechanism {
  private static final double SIM_LOOP_PERIOD = 0.004; // 4 ms
  private Notifier simNotifier = null;
  private double lastSimTime;

  /** 底盤所有裝置的警示（斷線、故障等），會顯示在儀表板。 */
  private final AlertableCollection deviceAlerts = new AlertableCollection("Swerve");

  /** 藍方駕駛的「前方」是 0 度（朝紅方聯盟牆）。 */
  private static final Rotation2d BLUE_ALLIANCE_FORWARD_DIRECTION = Rotation2d.ZERO;
  /** 紅方駕駛的「前方」是 180 度（朝藍方聯盟牆）。 */
  private static final Rotation2d RED_ALLIANCE_FORWARD_DIRECTION = Rotation2d.k180deg;
  /** 是否已經套用過駕駛視角。 */
  private boolean hasAppliedForwardDirection = false;

  // ---- 控制請求（SwerveRequest），建一次重複用，不要每圈 new ----
  /** 場地導向開迴路控制，跟 2026 一樣用 OpenLoopVoltage。 */
  private final SwerveRequest.FieldCentric fieldCentric =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /** 輪子擺成 X 字形煞車。 */
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  /** 不輸出（依照馬達設定的 neutral mode 煞車或滑行）。 */
  private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  /** 自動瞄準用的車頭角度 PID。 */
  private final PIDController headingController =
      new PIDController(DriverConstants.AIM_KP, DriverConstants.AIM_KI, DriverConstants.AIM_KD);

  /**
   * 建立底盤。不要自己 new，請用 {@code TunerConstants.createDrivetrain()}。
   *
   * @param drivetrainConstants 底盤共用常數
   * @param modules 四個模組的常數
   */
  public SwerveDrive(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    registerAlerts();
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  // ========================================================================
  // 指令（Commands）
  // ========================================================================

  /**
   * 預設指令：不輸出。
   *
   * <p>v3 的 Mechanism 本來就有 idle()，但預設版本「什麼都不做」。CTRE 的馬達會一直執行最後一個收到的控制請求，
   * 所以如果什麼都不做，車子會照著上一個速度一直跑！這裡覆寫成持續送 Idle 請求，讓馬達停下來。
   */
  @Override
  public Command idle() {
    return runRepeatedly(() -> setControl(idleRequest))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Swerve.Idle");
  }

  /**
   * Disabled 時讓馬達套用 neutral mode（照 CTRE 官方範例的做法）。
   *
   * <p>跟 {@link #idle()} 一樣送 Idle 請求，但優先權是一般值，才能打斷正在跑的駕駛指令。
   *
   * @return 指令
   */
  public Command neutralWhileDisabled() {
    return runRepeatedly(() -> setControl(idleRequest)).named("Swerve.DisabledNeutral");
  }

  /**
   * 駕駛手動控制（場地導向）。對應 2026 SwerveWithAim 沒按右肩鍵時的行為。
   *
   * <ul>
   *   <li>左搖桿：平移（平方曲線）
   *   <li>右搖桿 X：旋轉（線性）× 旋轉倍率
   * </ul>
   *
   * @param controller 駕駛手把
   * @param angularRateScale 旋轉倍率（從 Tunables 讀，可即時調）
   * @return 不會自己結束的指令
   */
  public Command driverControl(CommandGamepad controller, DoubleSupplier angularRateScale) {
    return runRepeatedly(
            () ->
                setControl(
                    fieldCentric
                        .withVelocityX(translationInput(-controller.getLeftY()))
                        .withVelocityY(translationInput(-controller.getLeftX()))
                        .withRotationalRate(
                            -controller.getRightX()
                                * DriverConstants.MAX_ANGULAR_RATE_RAD_PER_SEC
                                * angularRateScale.getAsDouble())))
        .named("Swerve.DriverControl");
  }

  /**
   * 自動瞄準：駕駛照樣控制平移，車頭自動對準目標點。對應 2026 SwerveWithAim 按住右肩鍵的行為。
   *
   * <p>瞄準好了而且駕駛沒有推左搖桿時，輪子擺 X 字形煞車（跟 2026 一樣）。
   *
   * <p>2026 是用 CTRE 的 FieldCentricFacingAngle，這裡改用 WPILib 的 PIDController 自己算旋轉速度，再交給
   * FieldCentric 請求。效果一樣，但每一步都看得到、好除錯。
   *
   * <p>沒有搬過來的部分：2026 的「移動中射擊補償」（ShotCompensator）需要射手的飛行時間表，等射手 Mechanism 做好再接。
   *
   * @param controller 駕駛手把
   * @param target 要對準的點（場地座標），每圈都會重新讀
   * @return 不會自己結束的指令（放開按鍵時被取消）
   */
  public Command aimAssist(CommandGamepad controller, Supplier<Translation2d> target) {
    return run(coroutine -> {
          // 每次按下都清掉上次殘留的 PID 狀態
          headingController.reset();

          while (true) {
            double vx = translationInput(-controller.getLeftY());
            double vy = translationInput(-controller.getLeftX());

            Pose2d pose = getState().Pose;
            // 2027 起 Translation2d.getAngle() 回傳 Optional：車子剛好在目標點上時角度沒有定義，
            // 這時就維持目前的車頭方向
            Rotation2d targetHeading =
                target.get().minus(pose.getTranslation()).getAngle().orElse(pose.getRotation());

            double omega =
                headingController.calculate(
                    pose.getRotation().getRadians(), targetHeading.getRadians());
            // 2027 的 MathUtil 拿掉了 clamp()，改用 Java 內建的 Math.clamp()
            omega =
                Math.clamp(
                    omega,
                    -DriverConstants.MAX_ANGULAR_RATE_RAD_PER_SEC,
                    DriverConstants.MAX_ANGULAR_RATE_RAD_PER_SEC);

            double headingError =
                MathUtil.angleModulus(targetHeading.getRadians() - pose.getRotation().getRadians());
            boolean aimed = Math.abs(headingError) < DriverConstants.AIM_TOLERANCE_RAD;
            boolean driverIsMoving = vx != 0.0 || vy != 0.0;

            if (aimed && !driverIsMoving) {
              setControl(brake);
            } else {
              setControl(
                  fieldCentric.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
            }

            coroutine.yield();
          }
        })
        .named("Swerve.AimAssist");
  }

  /**
   * 輪子擺 X 字形煞車，按住期間有效。
   *
   * @return 不會自己結束的指令
   */
  public Command xBrake() {
    return runRepeatedly(() -> setControl(brake)).named("Swerve.XBrake");
  }

  /**
   * 把「車子現在面對的方向」設成場地導向的正前方。對應 2026 左肩鍵的功能。
   *
   * @return 執行一次就結束的指令
   */
  public Command resetFieldCentricHeading() {
    return run(coroutine -> seedFieldCentric()).named("Swerve.ResetHeading");
  }

  // ========================================================================
  // 每圈更新與工具
  // ========================================================================

  /**
   * 每圈都要呼叫一次（Robot.robotPeriodic 裡）。內容對應 2026 CommandSwerveDrivetrain.periodic()。
   *
   * <p>依聯盟顏色設定駕駛視角：只在「還沒設定過」或「Disabled」時更新，避免比賽中途手感突然改變。
   */
  public void updateEachLoop() {
    if (!hasAppliedForwardDirection || RobotState.isDisabled()) {
      MatchState.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorForwardDirection(
                    allianceColor == Alliance.RED
                        ? RED_ALLIANCE_FORWARD_DIRECTION
                        : BLUE_ALLIANCE_FORWARD_DIRECTION);
                hasAppliedForwardDirection = true;
              });
    }

    deviceAlerts.report();
  }

  /**
   * 搖桿值（-1~1）轉成平移速度 (m/s)，套用保留正負號的次方曲線。
   *
   * @param stick 搖桿值，已經被 Gamepad 預設的 deadband 處理過
   * @return 速度 (m/s)
   */
  private static double translationInput(double stick) {
    return MathUtil.copyDirectionPow(stick, DriverConstants.TRANSLATION_INPUT_EXPONENT)
        * DriverConstants.MAX_SPEED_MPS;
  }

  @Override
  public void close() {
    /* close the sim notifier before closing the drivetrain */
    if (simNotifier != null) {
      simNotifier.close();
      simNotifier = null;
    }
    super.close();
  }

  private void registerAlerts() {
    /* register alerts for all the devices in the drivetrain */
    for (final var module : getModules()) {
      deviceAlerts
          .withAlertable(module.getDriveMotor())
          .withAlertable(module.getSteerMotor())
          .withAlertable(module.getEncoder());
    }
    deviceAlerts.withAlertable(getPigeon2());
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }
}

package first.robot;

import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;

import first.robot.constants.DriverConstants;
import first.robot.generated.TunerConstants;
import first.robot.mechanisms.SwerveDrive;
import first.robot.util.SwerveTelemetry;

/**
 * 機器人主類別（2027 OpMode + Commands v3 架構）。
 *
 * <p>跟 2026 的對照：
 *
 * <ul>
 *   <li>2026 {@code Robot extends TimedRobot} + {@code RobotContainer} → 2027 {@code Robot extends
 *       OpModeRobot}，硬體都放在這裡，不再需要 RobotContainer
 *   <li>2026 在 RobotContainer.configureBindings() 綁按鍵 → 2027 每個 OpMode（例如 {@code
 *       DriverTeleop}）在自己的建構子裡綁按鍵
 *   <li>2026 {@code CommandScheduler.getInstance().run()} → 2027 {@code Scheduler.getDefault().run()}
 *   <li>2026 {@code CommandXboxController} → 2027 {@code CommandGamepad}
 * </ul>
 *
 * <p>這裡放「不管哪個模式都成立」的東西：硬體、預設指令、Disabled 的行為、遙測。
 */
public class Robot extends OpModeRobot {
  /** 底盤（public 讓 OpMode 可以用）。 */
  public final SwerveDrive swerve = TunerConstants.createDrivetrain();

  /** 主駕駛手把（Xbox）。 */
  public final CommandGamepad driverController =
      new CommandGamepad(DriverConstants.DRIVER_CONTROLLER_PORT);

  /**
   * 旋轉倍率，可在儀表板上即時調（2026 的 SmartDashboard "AngularRate"）。
   *
   * <p>2027 起 SmartDashboard 被 Tunables 取代。
   */
  public final TunableDouble angularRateScale =
      Tunables.addDouble("Drive/AngularRateScale", DriverConstants.DEFAULT_ANGULAR_RATE_SCALE);

  private final SwerveTelemetry swerveTelemetry =
      new SwerveTelemetry(DriverConstants.MAX_SPEED_MPS);

  private final Scheduler scheduler = Scheduler.getDefault();

  public Robot() {
    // 沒有任何 OpMode 在跑時，底盤停住（覆寫過的 idle 會送 Idle 請求）
    swerve.setDefaultCommand(swerve.idle());

    // Disabled 時讓底盤馬達套用 neutral mode（2026 RobotContainer 裡的 "SwerveIdle"）
    RobotModeTriggers.disabled().whileTrue(swerve.neutralWhileDisabled());

    // 底盤狀態上傳到儀表板。CTRE 會在自己的里程計執行緒呼叫這個函式。
    swerve.registerTelemetry(swerveTelemetry::telemeterize);
  }

  @Override
  public void robotPeriodic() {
    // 1. 先更新底盤（駕駛視角、裝置警示），讓指令拿到最新狀態
    swerve.updateEachLoop();

    // 2. 跑排程器：檢查按鍵、執行指令
    scheduler.run();

    // 3. 其他遙測
    Telemetry.log("RobotVoltage", RobotController.getBatteryVoltage());
  }
}

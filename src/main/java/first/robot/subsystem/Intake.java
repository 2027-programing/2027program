package first.robot.subsystem;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

/**
 * Intake 子系統（Commands v3 Mechanism）。
 *
 * <p>一個 Mechanism 同時管理滾輪與手臂，因此三個命令互斥：同一時間只會有一個在執行。
 * 角度單位為「機構旋轉圈數（rotations）」，速度為 rotations/s，加速度為 rotations/s^2。
 */
public class Intake implements Mechanism {

  // ===== CAN =====
  private static final String kCanBusName = "GTX7130";
  private static final int kRollerId = 60;
  private static final int kArmId = 61;
  private static final int kCanCoderId = 62;

  // ===== 手臂角度（rotations）=====
  private static final double kAngleMax = -0.01;
  private static final double kAngleMin = -0.347412;

  // TODO（必須確認）：以下兩個目標角度你沒有提供數值，這裡暫用軟體極限邊界。
  // 請依實體機構確認哪一端是「放下吸球」、哪一端是「收起」，並必須落在 [kAngleMin, kAngleMax] 內。
  private static final double kIntakeAngle = kAngleMin;
  private static final double kDefaultAngle = kAngleMax;

  // ===== 滾輪電壓 =====
  private static final double kRollerVoltage = 5.76;

  // ===== Dynamic Motion Magic（TODO：你沒有提供，請依機構實測調整）=====
  private static final double kCruiseVelocity = 1.0; // rotations/s
  private static final double kAcceleration = 2.0; // rotations/s^2
  private static final double kJerk = 0.0; // 0 = 停用 jerk 限制

  // ===== Fused CANcoder 比例（TODO：你沒有提供，1.0 只是預設值，幾乎一定要改）=====
  private static final double kRotorToSensorRatio = 1.0; // 馬達轉子 : CANcoder
  private static final double kSensorToMechanismRatio = 1.0; // CANcoder : 手臂機構

  // ===== 硬體（全部 private）=====
  private final CANBus canBus = new CANBus(kCanBusName);
  private final TalonFX rollerMotor = new TalonFX(kRollerId, canBus);
  private final TalonFX armMotor = new TalonFX(kArmId, canBus);

  private final VoltageOut rollerVoltageRequest = new VoltageOut(0.0);
  private final NeutralOut rollerNeutralRequest = new NeutralOut();
  private final DynamicMotionMagicVoltage armRequest =
      new DynamicMotionMagicVoltage(kDefaultAngle, kCruiseVelocity, kAcceleration, kJerk);

  public Intake() {
    configureRoller();
    configureArm();
    setDefaultCommand(toDefaultState());
  }

  // ===================== 對外命令 =====================

  /** 手臂降至吸球角度，滾輪正轉。放開後由預設命令接手。 */
  public Command intake() {
    return runRepeatedly(() -> {
          setArmAngle(kIntakeAngle);
          setRollerVoltage(kRollerVoltage);
        })
        .named("Intake Intake");
  }

  /** 手臂降至吸球角度，滾輪反轉吐球。 */
  public Command eject() {
    return runRepeatedly(() -> {
          setArmAngle(kIntakeAngle);
          setRollerVoltage(-kRollerVoltage);
        })
        .named("Intake Eject");
  }

  /**
   * 預設狀態：手臂升至預設角度，滾輪停止（Coast）。
   *
   * <p>優先權必須在 .named() 之前設定（官方 v3 範例的寫法），預設命令使用最低優先權。
   */
  public Command toDefaultState() {
    return runRepeatedly(() -> {
          setArmAngle(kDefaultAngle);
          stopRoller();
        })
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Intake Default");
  }

  // ===================== 底層控制（private）=====================

  private void setArmAngle(double rotations) {
    armMotor.setControl(armRequest.withPosition(rotations));
  }

  private void setRollerVoltage(double volts) {
    rollerMotor.setControl(rollerVoltageRequest.withOutput(volts));
  }

  /** 使用 NeutralOut，才會真正套用 Coast；VoltageOut(0) 不是 Coast。 */
  private void stopRoller() {
    rollerMotor.setControl(rollerNeutralRequest);
  }

  // ===================== 設定 =====================

  private void configureRoller() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    applyWithRetry(rollerMotor, config, "Intake roller");
  }

  private void configureArm() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 25.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Fused CANcoder
    config.Feedback.FeedbackRemoteSensorID = kCanCoderId;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.RotorToSensorRatio = kRotorToSensorRatio;
    config.Feedback.SensorToMechanismRatio = kSensorToMechanismRatio;

    // Slot0：PID + 前饋 + 重力補償
    config.Slot0.kP = 16.0;
    config.Slot0.kI = 0.15;
    config.Slot0.kD = 0.57;
    config.Slot0.kG = 0.02;
    config.Slot0.kS = 0.02;
    config.Slot0.kV = 2.5;
    config.Slot0.kA = 0.02;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // 軟體極限
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kAngleMax;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kAngleMin;

    applyWithRetry(armMotor, config, "Intake arm");
  }

  private static void applyWithRetry(TalonFX motor, TalonFXConfiguration config, String name) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = motor.getConfigurator().apply(config);
      if (status.isOK()) {
        return;
      }
    }
    System.err.println(name + " 設定失敗: " + status);
  }
}
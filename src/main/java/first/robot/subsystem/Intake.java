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

import first.robot.Constants;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

/**
 * Intake 子系統（Commands v3 Mechanism）。
 *
 * 負責：
 * 1. 建立 Intake 使用的馬達並設定參數。
 * 2. 提供 intake / eject / toDefaultState 三個命令（互斥，因為屬於同一個 Mechanism）。
 *
 * runRepeatedly 只能在 Mechanism 內部呼叫，所以命令寫在這裡；
 * RobotCommands.java 只負責轉呼叫這些命令。
 */
public class Intake implements Mechanism {

    // ===================== 硬體 =====================

    private final CANBus canBus =
            new CANBus(Constants.CAN.CAN_BUS_NAME);

    private final TalonFX rollerMotor =
            new TalonFX(Constants.Intake.ROLLER_ID, canBus);

    private final TalonFX armMotor =
            new TalonFX(Constants.Intake.ARM_ID, canBus);


    // ===================== 控制請求 =====================

    // 滾輪電壓控制
    private final VoltageOut rollerVoltageRequest =
            new VoltageOut(0.0);

    // 滾輪停止控制
    private final NeutralOut rollerNeutralRequest =
            new NeutralOut();

    // 手臂 Motion Magic 控制
    private final DynamicMotionMagicVoltage armRequest =
            new DynamicMotionMagicVoltage(
                    Constants.Intake.DEFAULT_ANGLE,
                    Constants.Intake.CRUISE_VELOCITY,
                    Constants.Intake.ACCELERATION);


    // ===================== 建構子 =====================

    public Intake() {
        configureRoller();
        configureArm();

        // 預設命令（最低優先權已在 toDefaultState() 內設定）
        setDefaultCommand(toDefaultState());
    }


    // ===================== 對外命令 (Commands v3) =====================

    /** 手臂降至吸球角度，滾輪正轉。 */
    public Command intake() {
        return runRepeatedly(() -> {
            setArmAngle(Constants.Intake.INTAKE_ANGLE);
            setRollerVoltage(Constants.Intake.ROLLER_VOLTAGE);
        }).named("Intake Intake");
    }

    /** 手臂降至吸球角度，滾輪反轉吐球。 */
    public Command eject() {
        return runRepeatedly(() -> {
            setArmAngle(Constants.Intake.INTAKE_ANGLE);
            setRollerVoltage(-Constants.Intake.ROLLER_VOLTAGE);
        }).named("Intake Eject");
    }

    /** 預設狀態：手臂回預設角度，滾輪停止。優先權必須在 .named() 之前設定。 */
    public Command toDefaultState() {
        return runRepeatedly(() -> {
            setArmAngle(Constants.Intake.DEFAULT_ANGLE);
            stopRoller();
        })
                .withPriority(Command.LOWEST_PRIORITY)
                .named("Intake Default");
    }


    // ===================== 底層控制（private） =====================

    /**
     * 控制手臂移動到指定角度。
     *
     * @param rotations 目標角度，單位為 rotations
     */
    private void setArmAngle(double rotations) {
        armMotor.setControl(
                armRequest.withPosition(rotations));
    }

    /**
     * 設定滾輪輸出電壓。
     *
     * @param volts 輸出電壓
     */
    private void setRollerVoltage(double volts) {
        rollerMotor.setControl(
                rollerVoltageRequest.withOutput(volts));
    }

    /**
     * 停止滾輪。
     *
     * 使用 NeutralOut，保留原本的 Coast 行為。
     */
    private void stopRoller() {
        rollerMotor.setControl(rollerNeutralRequest);
    }


    // ===================== 滾輪設定 =====================

    private void configureRoller() {

        TalonFXConfiguration config =
                new TalonFXConfiguration();

        // 馬達方向與停止模式
        config.MotorOutput.Inverted =
                InvertedValue.CounterClockwise_Positive;

        config.MotorOutput.NeutralMode =
                NeutralModeValue.Coast;

        // 定子電流限制
        config.CurrentLimits.StatorCurrentLimit =
                Constants.Intake.ROLLER_STATOR_CURRENT_LIMIT;

        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // 電源電流限制
        config.CurrentLimits.SupplyCurrentLimit =
                Constants.Intake.ROLLER_SUPPLY_CURRENT_LIMIT;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        applyWithRetry(
                rollerMotor,
                config,
                "Intake roller");
    }


    // ===================== 手臂設定 =====================

    private void configureArm() {

        TalonFXConfiguration config =
                new TalonFXConfiguration();

        // 馬達方向與停止模式
        config.MotorOutput.Inverted =
                InvertedValue.Clockwise_Positive;

        config.MotorOutput.NeutralMode =
                NeutralModeValue.Brake;

        // 定子電流限制
        config.CurrentLimits.StatorCurrentLimit =
                Constants.Intake.ARM_STATOR_CURRENT_LIMIT;

        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // 電源電流限制
        config.CurrentLimits.SupplyCurrentLimit =
                Constants.Intake.ARM_SUPPLY_CURRENT_LIMIT;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;


        // ===== Fused CANcoder =====

        config.Feedback.FeedbackRemoteSensorID =
                Constants.Intake.CANCODER_ID;

        config.Feedback.FeedbackSensorSource =
                FeedbackSensorSourceValue.FusedCANcoder;

        config.Feedback.RotorToSensorRatio =
                Constants.Intake.ROTOR_TO_SENSOR_RATIO;

        config.Feedback.SensorToMechanismRatio =
                Constants.Intake.SENSOR_TO_MECHANISM_RATIO;


        // ===== PID、前饋與重力補償 =====

        config.Slot0.kP =
                Constants.Intake.KP;

        config.Slot0.kI =
                Constants.Intake.KI;

        config.Slot0.kD =
                Constants.Intake.KD;

        config.Slot0.kG =
                Constants.Intake.KG;

        config.Slot0.kS =
                Constants.Intake.KS;

        config.Slot0.kV =
                Constants.Intake.KV;

        config.Slot0.kA =
                Constants.Intake.KA;

        config.Slot0.GravityType =
                GravityTypeValue.Arm_Cosine;


        // ===== 軟體行程限制 =====

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable =
                true;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                Constants.Intake.ANGLE_MAX;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable =
                true;

        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                Constants.Intake.ANGLE_MIN;


        applyWithRetry(
                armMotor,
                config,
                "Intake arm");
    }


    // ===================== 設定重試 =====================

    private static void applyWithRetry(
            TalonFX motor,
            TalonFXConfiguration config,
            String name) {

        StatusCode status =
                StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; i++) {

            status = motor.getConfigurator().apply(config);

            if (status.isOK()) {
                return;
            }
        }

        System.err.println(
                name + " 設定失敗: " + status);
    }
}
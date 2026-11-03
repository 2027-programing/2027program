package first.robot.subsystem;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import first.robot.Constants;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

/**
 * Indexer 子系統（Commands v3 Mechanism）。
 *
 * <p>Up / Down 兩顆馬達屬於同一個 Mechanism，因此 feed / reverse / stop 三個命令互斥。
 *
 * <p>命令寫在 Mechanism 內（runRepeatedly 只能在 Mechanism 內部呼叫），
 * RobotCommands 只負責轉呼叫這些命令。固定數值統一放在 Constants.java。
 */
public class Indexer implements Mechanism {

  // ===== CAN =====
  private final CANBus canBus = new CANBus(Constants.CAN.CAN_BUS_NAME);

  // ===== 馬達 =====
  private final TalonFX upMotor = new TalonFX(Constants.Indexer.UP_MOTOR_ID, canBus);
  private final TalonFX downMotor = new TalonFX(Constants.Indexer.DOWN_MOTOR_ID, canBus);

  // ===== 控制請求 =====
  private final VoltageOut upRequest = new VoltageOut(0.0);
  private final VoltageOut downRequest = new VoltageOut(0.0);

  public Indexer() {
    // 預設命令（最低優先權已在 stop() 內設定）
    setDefaultCommand(stop());
  }

  // ===================== 對外命令 (Commands v3) =====================

  /** 進料：兩顆馬達正轉。 */
  public Command feed() {
    return runRepeatedly(() -> setVoltage(Constants.Indexer.INDEXING_VOLTAGE))
        .named("Indexer Feed");
  }

  /** 反轉排料：兩顆馬達反轉。 */
  public Command reverse() {
    return runRepeatedly(() -> setVoltage(-Constants.Indexer.INDEXING_VOLTAGE))
        .named("Indexer Reverse");
  }

  /** 停止：輸出 0 V。優先權必須在 .named() 之前設定。 */
  public Command stop() {
    return runRepeatedly(() -> setVoltage(0.0))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Indexer Stop");
  }

  // ===================== 底層控制（private）=====================

  /** 同時設定上下兩顆馬達的輸出電壓。 */
  private void setVoltage(double volts) {
    upMotor.setControl(upRequest.withOutput(volts));
    downMotor.setControl(downRequest.withOutput(volts));
  }
}
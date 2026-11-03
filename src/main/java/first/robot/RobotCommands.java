package first.robot;

import first.robot.subsystem.Indexer;
import first.robot.subsystem.Intake;
import org.wpilib.command3.Command;

/**
 * Robot 操作命令集中管理。
 *
 * <p>命令本身定義在各自的 Mechanism 內（Commands v3 的官方寫法：每個 Mechanism 對外提供具名命令方法），
 * 這個類別只負責把它們集中成一個入口，讓 Robot / OpMode 綁定按鍵時不用直接碰各 Mechanism 的細節。
 *
 * <p>Intake：intake() 吸入、eject() 排出、intakeDefault() 預設狀態。
 *
 * <p>Indexer：feed() 進料、reverse() 反轉排料、stopIndexer() 停止。
 *
 * <p>固定數值（角度、電壓）與硬體控制都在各自的 Mechanism 內，這裡不重複定義。
 */
public final class RobotCommands {

  private RobotCommands() {}

  // ===================== Intake =====================

  /** Intake 吸入：手臂到吸入位置，滾輪正轉。 */
  public static Command intake(Intake intake) {
    return intake.intake();
  }

  /** Intake 排出：手臂到吸入位置，滾輪反轉。 */
  public static Command eject(Intake intake) {
    return intake.eject();
  }

  /** Intake 預設狀態：手臂回預設位置、滾輪停止（最低優先權已在 Intake 內設定）。 */
  public static Command intakeDefault(Intake intake) {
    return intake.toDefaultState();
  }

  // ===================== Indexer =====================

  /** Indexer 進料：兩顆馬達正轉。 */
  public static Command feed(Indexer indexer) {
    return indexer.feed();
  }

  /** Indexer 反轉排料：兩顆馬達反轉。 */
  public static Command reverse(Indexer indexer) {
    return indexer.reverse();
  }

  /** Indexer 停止：輸出 0 V（最低優先權已在 Indexer 內設定）。 */
  public static Command stopIndexer(Indexer indexer) {
    return indexer.stop();
  }
}
package first.robot.opmodes.teleop;

import org.wpilib.command3.Command;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

import com.ctre.phoenix6.SignalLogger;

import first.robot.Robot;
import first.robot.constants.FieldConstants;

/**
 * 手動控制模式（Teleop OpMode）。在 2027 Driver Station 上選這個模式、按 Enable 就能開。
 *
 * <p>OpModeRobot 會自動掃描 {@code first.robot} 底下所有加了 {@code @Teleop} 的類別，不用手動註冊。
 *
 * <p>按鍵綁定寫在建構子裡，只在這個 OpMode 被選到的期間有效；換模式時會自動解除。
 *
 * <h2>按鍵配置（底盤部分，跟 2026 相同）</h2>
 *
 * <ul>
 *   <li>左搖桿：平移（場地導向）
 *   <li>右搖桿 X：旋轉
 *   <li>右肩鍵 (RB) 按住：自動瞄準（車頭對準目標，平移照樣自己控制）
 *   <li>左肩鍵 (LB)：重設場地導向的正前方
 *   <li>Start + Back 同時按：停止 CTRE SignalLogger 記錄
 * </ul>
 *
 * <p>2026 其他按鍵（LT 吸球、十字鍵、X/Y 爬升、POV 左測試射擊…）屬於 Intake / Climber / Shooter， 這些
 * Mechanism 還沒移植，所以先空著。
 *
 * <p>2027 Gamepad 按鍵改名：A/B/X/Y → faceDown/faceRight/faceLeft/faceUp。
 */
@Teleop(name = "Driver Teleop")
public class DriverTeleop implements OpMode {
  /**
   * OpModeRobot 會自動用目前的 Robot 物件呼叫這個建構子。
   *
   * @param robot 機器人
   */
  public DriverTeleop(Robot robot) {
    var swerve = robot.swerve;
    var driver = robot.driverController;

    // 預設：駕駛手動控制
    swerve.setDefaultCommand(swerve.driverControl(driver, robot.angularRateScale::get));

    // 右肩鍵按住：自動瞄準（放開就回到預設的手動控制）
    driver
        .rightBumper()
        .whileTrue(
            swerve.aimAssist(driver, () -> FieldConstants.aimTarget(swerve.getState().Pose)));

    // 左肩鍵：重設場地導向的正前方
    driver.leftBumper().onTrue(swerve.resetFieldCentricHeading());

    // Start + Back：停止 SignalLogger（2026 也有這個）
    driver
        .start()
        .and(driver.back())
        .onTrue(Command.noRequirements(coroutine -> SignalLogger.stop()).named("StopSignalLogger"));
  }
}

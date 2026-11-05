package first.robot.constants;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import first.robot.generated.TunerConstants;

/**
 * 駕駛操控相關常數。數值都照 2026 賽季程式的手感。
 *
 * <p>2027 命名規則：常數一律 ALL_CAPS（Alpha 7 起 WPILib 自己的常數也都改成全大寫）。
 */
public final class DriverConstants {
  private DriverConstants() {}

  /** 主駕駛 Xbox 手把在 Driver Station 上的編號（2026 是 0）。 */
  public static final int DRIVER_CONTROLLER_PORT = 0;

  /** 平移最高速度 (m/s)。2026 直接用 12 V 理論極速。 */
  public static final double MAX_SPEED_MPS = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  /** 旋轉基準速度 (rad/s) = 每秒 1 圈。實際轉速 = 這個值 × 旋轉倍率。 */
  public static final double MAX_ANGULAR_RATE_RAD_PER_SEC =
      RotationsPerSecond.of(1.0).in(RadiansPerSecond);

  /**
   * 旋轉倍率預設值。2026 是用 SmartDashboard 的 "AngularRate" 調（開機設成 1.5），
   * 2027 改用 Tunables，名稱 "Drive/AngularRateScale"，可以在儀表板上即時調。
   */
  public static final double DEFAULT_ANGULAR_RATE_SCALE = 1.5;

  /**
   * 搖桿曲線指數。2 = 平方（保留正負號），小推桿時比較好微調。2026 用的就是 2。
   *
   * <p>注意：2027 Alpha 7 的 Gamepad 已經「預設」幫搖桿套 0.1 的 deadband，這裡不要再套一次。
   */
  public static final double TRANSLATION_INPUT_EXPONENT = 2.0;

  // ---- 自動瞄準（按住右肩鍵）----
  /** 車頭角度 PID：輸入是角度誤差 (rad)，輸出是旋轉速度 (rad/s)。2026 的值是 P=12, D=0.3。 */
  public static final double AIM_KP = 12.0;

  public static final double AIM_KI = 0.0;
  public static final double AIM_KD = 0.3;

  /** 角度誤差小於這個值就算「瞄準好了」。2026 是 1.5 度。 */
  public static final double AIM_TOLERANCE_RAD = Degrees.of(1.5).in(Radians);
}

package first.robot.constants;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * 場地常數與藍/紅聯盟翻轉。
 *
 * <p>座標一律用「藍方原點」：場地左下角（藍方聯盟牆、右手邊）是 (0, 0)，+X 指向紅方。
 *
 * <p><b>!!! 以下全部是 2026 REBUILT 場地的數值 !!!</b> 2027 是新遊戲、新場地，賽季開始後一定要換掉。
 * 先保留 2026 的值，是為了讓「按住右肩鍵自動瞄準」這個功能可以在模擬裡先測試。
 */
public final class FieldConstants {
  private FieldConstants() {}

  /** 場地長 (m)。2026 PathPlanner 設定的值。 */
  public static final double FIELD_LENGTH_M = 16.54;

  /** 場地寬 (m)。 */
  public static final double FIELD_WIDTH_M = 8.07;

  /** 藍方 Hub 中心（2026）。 */
  public static final Translation2d HUB = new Translation2d(4.625, 4.034);

  /** 在中場時，往己方聯盟區「送球」的左右兩個目標點（2026）。 */
  public static final Translation2d LEFT_TRANSPORT_TARGET = new Translation2d(1.387, 6.487);

  public static final Translation2d RIGHT_TRANSPORT_TARGET = new Translation2d(1.387, 1.367);

  /** 己方聯盟區的深度（從聯盟牆往場內量）(m)。2026 的 4.528。 */
  public static final double ALLIANCE_ZONE_DEPTH_M = 4.528;

  /** 判斷左/右送球目標的分界線 y 值 (m)。 */
  public static final double FIELD_CENTER_Y_M = 4.035;

  /**
   * 目前是不是紅方。還沒連上 Driver Station（沒有聯盟資訊）時當作藍方。
   *
   * @return true 代表紅方
   */
  public static boolean isRedAlliance() {
    return MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED;
  }

  /**
   * 把「藍方座標」的點翻到紅方。2026 場地是旋轉對稱（繞場地中心轉 180 度）， 所以 x、y 都要翻。
   *
   * <p>2027 新場地如果是鏡像對稱，就只翻 x。
   *
   * @param bluePoint 藍方座標下的點
   * @return 紅方時回傳翻轉後的點，藍方時原樣回傳
   */
  public static Translation2d flipIfRed(Translation2d bluePoint) {
    if (!isRedAlliance()) {
      return bluePoint;
    }
    return new Translation2d(FIELD_LENGTH_M - bluePoint.getX(), FIELD_WIDTH_M - bluePoint.getY());
  }

  /**
   * 自動瞄準要對準哪一點（場地座標）。邏輯照 2026 的 SwerveWithAim：
   *
   * <ul>
   *   <li>車子在己方聯盟區裡 → 對準 Hub
   *   <li>不在己方聯盟區（在中場）→ 依照車子在左半邊或右半邊，對準左或右的送球點
   * </ul>
   *
   * @param robotPose 目前的車子位置（場地座標）
   * @return 要瞄準的點（場地座標）
   */
  public static Translation2d aimTarget(Pose2d robotPose) {
    boolean red = isRedAlliance();

    // 換算成「從自己聯盟牆看出去」的座標，左右判斷才會跟 2026 一樣
    double distanceFromOwnWall = red ? FIELD_LENGTH_M - robotPose.getX() : robotPose.getX();
    double allianceY = red ? FIELD_WIDTH_M - robotPose.getY() : robotPose.getY();

    Translation2d blueTarget;
    if (distanceFromOwnWall > 0 && distanceFromOwnWall < ALLIANCE_ZONE_DEPTH_M) {
      blueTarget = HUB;
    } else if (allianceY < FIELD_CENTER_Y_M) {
      blueTarget = RIGHT_TRANSPORT_TARGET;
    } else {
      blueTarget = LEFT_TRANSPORT_TARGET;
    }
    return flipIfRed(blueTarget);
  }
}

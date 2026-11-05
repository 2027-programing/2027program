package first.robot;

public final class Constants {

  private Constants() {}

  // =====================================
  // CAN Constants
  // =====================================
  public static final class CAN {
    public static final String CAN_BUS_NAME = "GTX7130";

    private CAN() {}
  }

  // =====================================
  // Intake Constants
  // =====================================
  public static final class Intake {

    // Motor IDs
    public static final int ROLLER_ID = 60;
    public static final int ARM_ID = 61;
    public static final int CANCODER_ID = 62;

    // Arm Angle (rotations)
    public static final double ANGLE_MAX = -0.01;
    public static final double ANGLE_MIN = -0.347412;

    public static final double INTAKE_ANGLE = ANGLE_MIN;
    public static final double DEFAULT_ANGLE = ANGLE_MAX;

    // Roller Voltage
    public static final double ROLLER_VOLTAGE = 5.76;

    // Motion Magic
    public static final double CRUISE_VELOCITY = 1.0;
    public static final double ACCELERATION = 2.0;

    // CANcoder Ratios
    public static final double ROTOR_TO_SENSOR_RATIO = 1.0;
    public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;

    // Current Limits
    public static final double ROLLER_STATOR_CURRENT_LIMIT = 80.0;
    public static final double ROLLER_SUPPLY_CURRENT_LIMIT = 40.0;

    public static final double ARM_STATOR_CURRENT_LIMIT = 60.0;
    public static final double ARM_SUPPLY_CURRENT_LIMIT = 25.0;

    // PID + Feedforward
    public static final double KP = 16.0;
    public static final double KI = 0.15;
    public static final double KD = 0.57;

    public static final double KG = 0.02;
    public static final double KS = 0.02;
    public static final double KV = 2.5;
    public static final double KA = 0.02;

    private Intake() {}
  }

  // =====================================
  // Indexer Constants
  // =====================================
  public static final class Indexer {

    // Motor IDs
    public static final int UP_MOTOR_ID = 44;
    public static final int DOWN_MOTOR_ID = 43;

    // Indexer Output
    public static final double NOMINAL_VOLTAGE = 12.0;
    public static final double INDEXING_SPEED = 0.63;

    public static final double INDEXING_VOLTAGE =
        NOMINAL_VOLTAGE * INDEXING_SPEED;

    private Indexer() {}
  }
}
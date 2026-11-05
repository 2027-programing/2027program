// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first;

import org.wpilib.framework.RobotBase;

/**
 * 程式進入點。不要在這裡加任何 static 變數或初始化。
 *
 * <p>2027 Alpha 7 的寫法是 {@code Robot::new}（工廠函式），不是舊 Alpha 的 {@code Robot.class}。
 */
public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(first.robot.Robot::new);
  }
}

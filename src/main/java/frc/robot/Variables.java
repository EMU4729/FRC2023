// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

/**
 * Variables - use this class to define and set globally-accessed
 * settings/options/variables.
 */
public final class Variables {
  private static Optional<Variables> inst = Optional.empty();
  public static Variables getInstance() {
    if (!inst.isPresent()) inst = Optional.of(new Variables());
    return inst.get();
  }


  /**
   * Bool to invert robot drive direction flipping the apparent front of the robot
   */
  public boolean invertDriveDirection = false;

  public double test1Speed = 0.5;
  public double test2Speed = 0.5;

  /** max speed of robot m/s */
  public double robotMaxSpeed = 3.850;
  /** min throttle for movement */
  public double robotminThrot = 0.3;
  /** min throttle for turning */
  public double robotminTurn = 0.3;
  /**
   * settings for robot drive in default teleop
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve power}
   */
  public double[][] DriveSettingsTELEOP = { { robotminThrot, 1, 3 }, { robotminTurn, 1, 3, 0.3 } };
  /**
   * settings for robot drive in demo mode
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve power}
   */
  public double[][] DriveSettingsDEMO = { { robotminThrot, 0.5, 3 }, { robotminTurn, 0.6, 3, 0.1 } };
  /**
   * settings for robot drive in PID drive
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve power}
   */
  public double[][] DriveSettingsPID1 = { { 0, robotMaxSpeed, 3 }, { 0, 1, 3, 0.3 } };
  /**
   * settings for robot drive in PID drive
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve power}
   */
  public double[][] DriveSettingsPID2 = { { robotminThrot, 1, 1 }, { robotminTurn, 1, 1, 0.3 } };

  /** LED string brightness modifier (0-1) */
  public double LEDBrightnessMod = 1;
}

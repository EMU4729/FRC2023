// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Variables - use this class to define and set globally-accessed
 * settings/options/variables.
 */
public final class Variables {
  /**
   * Bool to invert robot drive direction flipping the apparent front of the robot
   */
  public static boolean invertDriveDirection = false;

  /** max speed of robot m/s */
  public static double driveMaxSpeed = 3.850;
  /** min throttle for movement */
  public static double driveMinThrot = 0.3;
  /** min throttle for turning */
  public static double driveMinTurn = 0.3;
  /**
   * settings for robot drive in default teleop
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve
   * power}
   */
  public static double[][] pilotDriveSettings = { { driveMinThrot, 1, 2 }, { driveMinTurn, 1, 3, 0.3 } };
  /**
   * settings for robot drive in demo mode
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve
   * power}
   */
  public static double[][] demoDriveSettings = { { driveMinThrot, 0.5, 3 }, { driveMinTurn, 0.6, 3, 0.1 } };

  /**
   * settings for copilot drive
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve
   * power}
   */
  public static double[][] copilotDriveSettings = { { driveMinThrot, 0.5, 3 }, { driveMinTurn, 0.6, 3, 0.1 } };

  /**
   * settings for robot drive in PID drive
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve
   * power}
   */
  public static double[][] pid1DriveSettings = { { 0, driveMaxSpeed, 3 }, { 0, 1, 3, 0.3 } };
  /**
   * settings for robot drive in PID drive
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve
   * power}
   */
  public static double[][] pid2DriveSettings = { { driveMinThrot, 1, 1 }, { driveMinTurn, 1, 1, 0.3 } };

  /** LED string brightness modifier (0-1) */
  public static double ledBrightnessMod = 1;
}

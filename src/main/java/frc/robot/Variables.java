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

  private Variables() {
  }

  public static Variables getInstance() {
    if (!inst.isPresent()) {
      inst = Optional.of(new Variables());
    }
    return inst.get();
  }

  /** Bool to invert robot steering direction */
  public boolean invertSteering = false;
  /**
   * Bool to invert robot drive direction flipping the apparent front of the robot
   */
  public boolean invertDriveDirection = false;

  public double test1Speed = 0.5;
  public double test2Speed = 0.5;

  /** Multiplier for robot max speed in teleop */
  public double teleopSpeedMultiplier = 1;
  /** Multiplier for robot max speed in auto */
  public double autoSpeedMultiplier = 1;

  /** Drive Speed Curve Exponent */
  public double speedCurveExponent = 3;
  /** Drive Turning Curve Exponent */
  public double turnCurveExponent = 3;

  /** max speed of robot m/s */
  public double robotMaxSpeed = 3.850;
  /** seconds 0 -> max */
  public int accelTime = 5;
  private double accelSteps = accelTime / 0.02;
  public double accelInterval = 1 / accelSteps;
  /** min throttle for movement */
  public double robotminThrot = 0.3;
  /** min throttle for turning */
  public double robotminTurn = 0.3;
  /**
   * settings for robot drive in default teleop
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve
   * power,throttle effect}
   */
  public double[][] DriveSettingsTELEOP = { { robotminThrot, 1, 3 }, { robotminTurn, 1, 3, 0.3 } };
  /**
   * settings for robot drive in demo mode
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve
   * power}
   */
  public double[][] DriveSettingsDEMO = { { robotminThrot, 0.5, 3 }, { robotminTurn, 0.6, 3, 0.1 } };
  /**
   * settings for robot drive in PID drive
   * {min speed,max speed,curve power}, {min turn rate , max turn rate,curve
   * power}
   */
  public double[][] DriveSettingsPID1 = { { 0, robotMaxSpeed, 3 }, { 0, 1, 3, 0.3 } };
  /**
   * settings for robot drive in PID drive
   * {min speed,max speed,curve power}, {min turn rate , max turn rate,curve
   * power}
   */
  public double[][] DriveSettingsPID2 = { { robotminThrot, 1, 1 }, { robotminTurn, 1, 1, 0.3 } };

  /** should max speed be updated if the robot exedes it */
  public boolean autoUpdateMaxSpeed = false;

}

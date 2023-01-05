package frc.robot;

import java.util.Optional;

import frc.robot.auto.AutoDriveStraight;

/**
 * Commands - Use this class to initialize and access commands globally.
 */
public class Commands {
  public Commands() {}

  public static final AutoDriveStraight autoDriveStraight = new AutoDriveStraight();
}

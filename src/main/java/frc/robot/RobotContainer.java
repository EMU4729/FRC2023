// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.AutoProvider;
import frc.robot.teleop.TeleopProvider;
import frc.robot.utils.LEDControl.LEDControl;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including
 * Subsystemsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final AutoProvider autoProvider = AutoProvider.getInstance();
  private final TeleopProvider teleopProvider = TeleopProvider.getInstance();

  /**
   * The container for the robot. Contains Subsystemsystems, OI devices, and
   * commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // +----------------+
    // | PILOT CONTROLS |
    // +----------------+

    // Invert Drive
    LEDControl.getInstance().runDirectionLights();
    OI.pilot.start().onTrue(new InstantCommand(() -> {
      Variables.invertDriveDirection = !Variables.invertDriveDirection;
      LEDControl.getInstance().runDirectionLights();
      // LEDPattern.runDirLEDS();
    }));

    OI.pilot.rightBumper().onTrue(new InstantCommand(() -> {
      LEDControl.getInstance().runConeLights();
    }));
    OI.pilot.leftBumper().onTrue(new InstantCommand(() -> {
      LEDControl.getInstance().runCubeLights();
    }));

    // Drive bindings handled in teleop command

    // +------------------+
    // | COPILOT CONTROLS |
    // +------------------+

    // Calibration
    OI.copilot.start().onTrue(
        new InstantCommand(Subsystems::calibrate, Subsystems.arm, Subsystems.subArmPivot, Subsystems.subArmRotate));

    // Gripper Control
    // OI.copilot.a().onTrue(new InstantCommand(Subsystems.gripper::open, Subsystems.gripper));
    // OI.copilot.b().onTrue(new InstantCommand(Subsystems.gripper::closeCube, Subsystems.gripper));
    // OI.copilot.y().onTrue(new InstantCommand(Subsystems.gripper::closeCone, Subsystems.gripper));

    // Arm controls handled in ArmSub
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getTeleopCommand() {
    return teleopProvider.getSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoProvider.getSelected();
  }
}

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

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including
 * Subsystemsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Variables vars = Variables.getInstance();
  private final Constants cnst = Constants.getInstance();
  private final AutoProvider autoProvider = AutoProvider.getInstance();
  private final TeleopProvider teleopProvider = TeleopProvider.getInstance();
  private final OI oi = OI.getInstance();

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
    // Invert Drive

    // oi.pilot.start().onTrue(new InstantCommand(() -> {
    // vars.invertDriveDirection = !vars.invertDriveDirection;
    // // LEDPattern.runDirLEDS();
    // }));

    // Calibration
    oi.pilot.start().onTrue(new InstantCommand(Subsystems.arm::calibrate, Subsystems.arm));

    // D-Pad Arm Bindings
    oi.pilot.povUp().onTrue(Subsystems.arm.farField());
    oi.pilot.povRight().onTrue(Subsystems.arm.lowField());
    oi.pilot.povDown().onTrue(Subsystems.arm.lowerRung());
    oi.pilot.povLeft().onTrue(Subsystems.arm.upperRung());

    // Fine arm movement bindings
    oi.pilot.axisGreaterThan(XboxController.Axis.kRightX.value, 0.8).whileTrue(Subsystems.arm.moveForward());
    oi.pilot.axisLessThan(XboxController.Axis.kRightX.value, -0.8).whileTrue(Subsystems.arm.moveBack());
    oi.pilot.axisGreaterThan(XboxController.Axis.kRightY.value, 0.8).whileTrue(Subsystems.arm.moveUp());
    oi.pilot.axisLessThan(XboxController.Axis.kRightY.value, -0.8).whileTrue(Subsystems.arm.moveDown());

    // Arm Invert - DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS
    // DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE
    // THIS DO NOT USE THIS DO NOT USE THIS
    oi.pilot.b().onTrue(new InstantCommand(Subsystems.arm::invert, Subsystems.arm));

    // Drive bindings handled in teleop command
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getTeleopCommand() {
    return teleopProvider.getTeleop();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoProvider.getMiddleAuto();
  }
}

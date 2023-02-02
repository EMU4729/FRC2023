package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SubArmPivotSub extends SubsystemBase {
  private final Constants cnst = Constants.getInstance();

  private final MotorController motor = cnst.SUBARM_ROTATE_MOTOR_ID.createMotorController();
  private final Encoder encoder = cnst.SUBARM_ROTATE_MOTOR_ID.createEncoder();
}

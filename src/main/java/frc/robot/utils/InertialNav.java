package frc.robot.utils;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SPI;

//https://digital-library.theiet.org/content/books/ra/pbra017e

public class InertialNav extends ADIS16470_IMU {

  private double localGravity = 9.796;//rough initial value (m/s)

  /** acceleration in the direction of travel forwards/backwards rel to the robot (m/s) */
  private double xAccelBot(){ return super.getAccelX(); }
  /** acceleration left/right rel to the robot (m/s) */
  private double yAccelBot(){ return super.getAccelY(); }
  /** acceleration up/down rel to the robot (m/s) */
  private double zAccelBot(){ return super.getAccelZ(); }
  /** angle in pitch of the robot (deg) */
  private double pitchBot(){ return super.getYComplementaryAngle(); }
  /** angle in roll of the robot (deg) */
  private double rollBot(){ return super.getXComplementaryAngle(); }
  /** angle in yaw of the robot (deg) */
  private double yawBot(){ return super.getAngle(); }

  


  public InertialNav(){
    super();

  }
  public InertialNav(IMUAxis yaw_axis, SPI.Port port, CalibrationTime cal_time){
    super(yaw_axis, port, cal_time);
  }
  

  public void setPos(){}

  @Override
  public void reset(){
    super.reset();
  }

  /** Sets local gravity for compensation
   *  Robot must be upright and stationary on flat ground 
   */
  public void calibrateGrav(){ localGravity = zAccelBot();}
}

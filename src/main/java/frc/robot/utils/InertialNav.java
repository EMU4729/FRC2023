package frc.robot.utils;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SPI;

public class InertialNav extends ADIS16470_IMU {
  private double pitch(){ return super.getYComplementaryAngle();}
  private double roll(){ return super.getXComplementaryAngle();}
  private double yaw(){ return super.getAngle();}
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

  static matMult
}

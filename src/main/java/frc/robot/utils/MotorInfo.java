package frc.robot.utils;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.utils.logger.Logger;

public class MotorInfo {
  public final int motorPort;
  public final Type type;
  public boolean invertMotor = false;
  public boolean invertEncoder = false;
  public boolean brake = false;
  public boolean safety = false;
  public Optional<int[]> encoderPort = Optional.empty();
  private Optional<Double> encoderSteps = Optional.empty();

  public enum Type {
    Never,
    TalonSRX,
    VictorSPX
  }

  public MotorInfo(int motorPort, Type type) {
    this.motorPort = motorPort;
    this.type = type;
  }

  public MotorInfo encoder(int[] encoderPort, double encoderSteps) {
    this.encoderPort = Optional.of(encoderPort);
    this.encoderSteps = Optional.of(encoderSteps);
    return this;
  }

  public MotorInfo withBrake() {
    brake = true;
    return this;
  }

  /** inverts both motor and encodor */
  public MotorInfo withInvert() {
    invertMotor = !invertMotor;
    invertEncoder = !invertEncoder;
    return this;
  }

  public MotorInfo withSafety() {
    safety = true;
    return this;
  }

  public MotorInfo withInvertedEncodor() {
    invertEncoder = !invertEncoder;
    return this;
  }

  public MotorController createMotorController() {
    if (motorPort < 0) {
      Logger.error("MotorInfo : motor port num < 0, check port is defined : " + motorPort);
      return new WPI_TalonSRX(99);
    }
    switch (type) {
      case TalonSRX:
        WPI_TalonSRX talon = new WPI_TalonSRX(motorPort);
        if (!talon.isAlive()) {
          Logger.warn(
              "MotorInfo : new WPI_TalonSRX on port " + motorPort + "not found, may not exist or be of wrong type");
        }
        talon.setInverted(invertMotor);
        if (brake) {
          talon.setNeutralMode(NeutralMode.Brake);
        } else {
          talon.setNeutralMode(NeutralMode.Coast);
        }
        talon.setSafetyEnabled(safety);
        return talon;
      case VictorSPX:
        WPI_VictorSPX victor = new WPI_VictorSPX(motorPort);
        if (!victor.isAlive()) {
          Logger.warn(
              "MotorInfo : new WPI_VictorSPX on port " + motorPort + "not found, may not exist or be of wrong type");
        }
        victor.setInverted(invertMotor);
        if (brake) {
          victor.setNeutralMode(NeutralMode.Brake);
        } else {
          victor.setNeutralMode(NeutralMode.Coast);
        }
        victor.setSafetyEnabled(safety);
        return victor;
      case Never:
        Logger.error("MotorInfo : controller type not set");
        return new WPI_TalonSRX(99);
      default:
        Logger.error("MotorInfo : controller type not found");
        return new WPI_TalonSRX(99);
    }
  }

  public Encoder createEncoder() {
    if (encoderPort.isPresent()) {
      if (encoderPort.get()[0] < 0) {
        throw new IllegalStateException("MotorInfo : encoder port 1 < 0, check port is setup");
      }
      if (encoderPort.get()[1] < 0) {
        throw new IllegalStateException("MotorInfo : encoder port 2 < 0, check port is setup");
      }
    } else {
      throw new IllegalStateException("MotorInfo : EncoderPort not found, check EncoderPort is defined");
    }
    if (encoderSteps.isPresent()) {
      if (encoderSteps.get() < 0) {
        throw new IllegalArgumentException("MotorInfo : EncoderSteps < 0, check EncoderSteps is setup");
      }
    } else {
      throw new IllegalStateException("MotorInfo : EncoderSteps not found, check EncoderSteps is defined");
    }

    Encoder encoder = new Encoder(encoderPort.get()[0], encoderPort.get()[1], invertEncoder, Encoder.EncodingType.k2X);
    encoder.setDistancePerPulse(encoderSteps.get());
    encoder.setMinRate(0.1 * encoderSteps.get());
    encoder.setMinRate(10);
    encoder.setSamplesToAverage(5);
    return encoder;
  }
}

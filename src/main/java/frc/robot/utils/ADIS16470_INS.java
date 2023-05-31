// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

// import java.lang.FdLibm.Pow;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.shufflecontrol.CalibrationTab;

import java.nio.Buffer;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

// CHECKSTYLE.OFF: TypeName
// CHECKSTYLE.OFF: MemberName
// CHECKSTYLE.OFF: SummaryJavadoc
// CHECKSTYLE.OFF: UnnecessaryParentheses
// CHECKSTYLE.OFF: OverloadMethodsDeclarationOrder
// CHECKSTYLE.OFF: NonEmptyAtclauseDescription
// CHECKSTYLE.OFF: MissingOverride
// CHECKSTYLE.OFF: AtclauseOrder
// CHECKSTYLE.OFF: LocalVariableName
// CHECKSTYLE.OFF: RedundantModifier
// CHECKSTYLE.OFF: AbbreviationAsWordInName
// CHECKSTYLE.OFF: ParameterName
// CHECKSTYLE.OFF: EmptyCatchBlock
// CHECKSTYLE.OFF: MissingJavadocMethod
// CHECKSTYLE.OFF: MissingSwitchDefault
// CHECKSTYLE.OFF: VariableDeclarationUsageDistance
// CHECKSTYLE.OFF: ArrayTypeStyle

/** This class is for the ADIS16470 IMU that connects to the RoboRIO SPI port. */
@SuppressWarnings({
  "unused",
  "PMD.RedundantFieldInitializer",
  "PMD.ImmutableField",
  "PMD.SingularField",
  "PMD.CollapsibleIfStatements",
  "PMD.MissingOverride",
  "PMD.EmptyIfStmt",
  "PMD.EmptyStatementNotInLoop"
})
public class ADIS16470_INS implements AutoCloseable, NTSendable {
  /* ADIS16470 Register Map Declaration */
  private static final int FLASH_CNT = 0x00; // Flash memory write count
  private static final int DIAG_STAT = 0x02; // Diagnostic and operational status
  // 7 bit - Clock Sync Error (clock has not synced with external clock)
  // 6 bit - Memory Failure (memory checksum error)(hope you dont see this)
  // 5 bit - Sensor Failure (1+ gyro/accel fail internal test)
  // 4 bit - Low Power Standby (supply voltage too low)(you shouldn't see this)
  // 3 bit - SPI Comms Error (error in the comms system behind the sceens)
  // 2 bit - Flash Memory Update Failure (prod wont see this)
  // 1 bit - Data Path Overrun (something internal overran restart the imu and go again)
  private static final int X_GYRO_LOW = 0x04; // X-axis gyroscope output, lower word
  private static final int X_GYRO_OUT = 0x06; // X-axis gyroscope output, upper word
  private static final int Y_GYRO_LOW = 0x08; // Y-axis gyroscope output, lower word
  private static final int Y_GYRO_OUT = 0x0A; // Y-axis gyroscope output, upper word
  private static final int Z_GYRO_LOW = 0x0C; // Z-axis gyroscope output, lower word
  private static final int Z_GYRO_OUT = 0x0E; // Z-axis gyroscope output, upper word
  private static final int X_ACCL_LOW = 0x10; // X-axis accelerometer output, lower word
  private static final int X_ACCL_OUT = 0x12; // X-axis accelerometer output, upper word
  private static final int Y_ACCL_LOW = 0x14; // Y-axis accelerometer output, lower word
  private static final int Y_ACCL_OUT = 0x16; // Y-axis accelerometer output, upper word
  private static final int Z_ACCL_LOW = 0x18; // Z-axis accelerometer output, lower word
  private static final int Z_ACCL_OUT = 0x1A; // Z-axis accelerometer output, upper word
  private static final int TEMP_OUT = 0x1C; // Temperature output (internal, not calibrated)
  private static final int TIME_STAMP = 0x1E; // PPS mode time stamp
  private static final int X_DELTANG_LOW = 0x24; // X-axis delta angle output, lower word
  private static final int X_DELTANG_OUT = 0x26; // X-axis delta angle output, upper word
  private static final int Y_DELTANG_LOW = 0x28; // Y-axis delta angle output, lower word
  private static final int Y_DELTANG_OUT = 0x2A; // Y-axis delta angle output, upper word
  private static final int Z_DELTANG_LOW = 0x2C; // Z-axis delta angle output, lower word
  private static final int Z_DELTANG_OUT = 0x2E; // Z-axis delta angle output, upper word
  private static final int X_DELTVEL_LOW = 0x30; // X-axis delta velocity output, lower word
  private static final int X_DELTVEL_OUT = 0x32; // X-axis delta velocity output, upper word
  private static final int Y_DELTVEL_LOW = 0x34; // Y-axis delta velocity output, lower word
  private static final int Y_DELTVEL_OUT = 0x36; // Y-axis delta velocity output, upper word
  private static final int Z_DELTVEL_LOW = 0x38; // Z-axis delta velocity output, lower word
  private static final int Z_DELTVEL_OUT = 0x3A; // Z-axis delta velocity output, upper word
  private static final int XG_BIAS_LOW  = 0x40; // X-axis gyroscope bias offset correction, lower word
  private static final int XG_BIAS_HIGH = 0x42; // X-axis gyroscope bias offset correction, upper word
  private static final int YG_BIAS_LOW  = 0x44; // Y-axis gyroscope bias offset correction, lower word
  private static final int YG_BIAS_HIGH = 0x46; // Y-axis gyroscope bias offset correction, upper word
  private static final int ZG_BIAS_LOW  = 0x48; // Z-axis gyroscope bias offset correction, lower word
  private static final int ZG_BIAS_HIGH = 0x4A; // Z-axis gyroscope bias offset correction, upper word
  private static final int XA_BIAS_LOW  = 0x4C; // X-axis accelerometer bias offset correction, lower word
  private static final int XA_BIAS_HIGH = 0x4E; // X-axis accelerometer bias offset correction, upper word
  private static final int YA_BIAS_LOW  = 0x50; // Y-axis accelerometer bias offset correction, lower word
  private static final int YA_BIAS_HIGH = 0x52; // Y-axis accelerometer bias offset correction, upper word
  private static final int ZA_BIAS_LOW  = 0x54; // Z-axis accelerometer bias offset correction, lower word
  private static final int ZA_BIAS_HIGH = 0x56; // Z-axis accelerometer bias offset correction, upper word
  private static final int FILT_CTRL = 0x5C; // Filter control
  private static final int MSC_CTRL = 0x60; // Miscellaneous control
  private static final int UP_SCALE = 0x62; // Clock scale factor, PPS mode
  private static final int DEC_RATE = 0x64; // Decimation rate control (output data rate)
  private static final int NULL_CNFG = 0x66; // Auto-null configuration control
  private static final int GLOB_CMD = 0x68; // Global commands
  private static final int FIRM_REV = 0x6C; // Firmware revision
  private static final int FIRM_DM = 0x6E; // Firmware revision date, month and day
  private static final int FIRM_Y = 0x70; // Firmware revision date, year
  private static final int PROD_ID = 0x72; // Product identification
  private static final int SERIAL_NUM = 0x74; // Serial number (relative to assembly lot)
  private static final int USER_SCR1 = 0x76; // User scratch register 1
  private static final int USER_SCR2 = 0x78; // User scratch register 2
  private static final int USER_SCR3 = 0x7A; // User scratch register 3
  private static final int FLSHCNT_LOW = 0x7C; // Flash update count, lower word
  private static final int FLSHCNT_HIGH = 0x7E; // Flash update count, upper word

  private static final byte[] autoSPIPacket = {
    X_DELTANG_OUT,
    X_DELTANG_LOW,
    Y_DELTANG_OUT,
    Y_DELTANG_LOW,
    Z_DELTANG_OUT,
    Z_DELTANG_LOW,

    X_DELTVEL_OUT,
    X_DELTVEL_LOW,
    Y_DELTVEL_OUT,
    Y_DELTVEL_LOW,
    Z_DELTVEL_OUT,
    Z_DELTVEL_LOW,

    X_ACCL_OUT,
    Y_ACCL_OUT,
    Z_ACCL_OUT,

    DIAG_STAT
  };

  public enum CalibrationTime {
    _32ms(0),
    _64ms(1),
    _128ms(2),
    _256ms(3),
    _512ms(4),
    _1s(5),
    _2s(6),
    _4s(7),
    _8s(8),
    _16s(9),
    _32s(10),
    _64s(11);

    private int value;

    private CalibrationTime(int value) {
      this.value = value;
    }

    private boolean equals(CalibrationTime calTime){
      return value == calTime.value;
    }
  }

  public enum IMUAxis {
    X_CW(1),
    X_ACW(-1),
    Y_CW(2),
    Y_ACW(-2),
    Z_CW(3),
    Z_ACW(-3);

    private int value;
    
    private IMUAxis(int value){
        this.value = value;
    }

    private boolean equal(IMUAxis cmp){
      return Math.abs(value) == Math.abs(cmp.value);
    }
    private int direction(){
      return Integer.signum(value);
    }

    private int axis(){
      return Math.abs(value);
    }
  }

  // Static Constants
  //data sheet page 22 table 59, conversion from responce to number
  private static final double MULT_GYRO_ANGLE_SF_16  = 0.1;
  private static final double MULT_GYRO_ANGLE_SF_32  = 0.1 / 65536; /* 0.1 / 2^16 */
  private static final double MULT_ACCEL_SF_16       = 0.00125; /* 0.125 / 1000 */
  private static final double MULT_ACCEL_SF_32       = 0.00125 / 65536; /* 0.125 / 1000 / 2^16 */
  private static final double MULT_DELTA_ANGLE_SF_16 = 2160.0 / 2147483648.0; /* 2160 / (2^31) */
  private static final double MULT_DELTA_ANGLE_SF_32 = 2160.0 / 2147483648.0; /* 2160 / (2^31) */
  private static final double MULT_DELTA_VEL_SF_16   = 400.0 / 2147483648.0; /*  400 / (2^31) */
  private static final double MULT_DELTA_VEL_SF_32   = 400.0 / 2147483648.0; /*  400 / (2^31) */
  private static final double localGravity = 9.81;

  // calculation variables
  private double m_dt = 0.0;

  // User-specified IMU axes
  private IMUAxis robotPitch;
  private IMUAxis robotRoll;
  private IMUAxis robotYaw;

  // Estimated Robot Angular and Linear Acceleration
  private double RobotAngularAccelX = 0.0; // deg/s^2
  private double RobotAngularAccelY = 0.0; // deg/s^2
  private double RobotAngularAccelZ = 0.0; // deg/s^2

  private double RobotLinearAccelX = 0.0; // m/s^2
  private double RobotLinearAccelY = 0.0; // m/s^2
  private double RobotLinearAccelZ = 0.0; // m/s^2

  // Estimated Robot Angular and Linear Velocity
  private double RobotAngularVelX = 0.0; // deg/s
  private double RobotAngularVelY = 0.0; // deg/s
  private double RobotAngularVelZ = 0.0; // deg/s

  private double RobotLinearVelX = 0.0;  // m/s
  private double RobotLinearVelY = 0.0;  // m/s
  private double RobotLinearVelZ = 0.0;  // m/s

  // Estimated Robot Angular and Linear Posiotion In Relation to Field 0,0,0 pos and 0,0,0 angle
  private double FieldAngularPosX = 0.0; // deg/s
  private double FieldAngularPosY = 0.0; // deg/s
  private double FieldAngularPosZ = 0.0; // deg/s

  private double FieldLinearPosX = 0.0;  // m/s
  private double FieldLinearPosY = 0.0;  // m/s
  private double FieldLinearPosZ = 0.0;  // m/s

  // Debug values from the imu
  public int IMUDebugOut = 0; 

  // State variables
  private volatile boolean m_thread_active = false;
  private CalibrationTime calibrationTime;
  private volatile boolean m_first_run = true;
  private volatile boolean m_thread_idle = false;
  private boolean m_auto_configured = false;
  private double m_scaled_sample_rate = 2500.0;

  // Resources
  private SPI spi;
  private SPI.Port spiPort;
  private DigitalInput m_auto_interrupt;
  private DigitalOutput IMUStatusLed;
  private DigitalInput resetInput;
  private Thread m_acquire_task;
  private boolean IMUConnected;

  private SimDevice simDevice;
  private SimBoolean m_simConnected;
  private SimDouble m_simGyroAngleX;
  private SimDouble m_simGyroAngleY;
  private SimDouble m_simGyroAngleZ;
  private SimDouble m_simGyroRateX;
  private SimDouble m_simGyroRateY;
  private SimDouble m_simGyroRateZ;
  private SimDouble m_simAccelX;
  private SimDouble m_simAccelY;
  private SimDouble m_simAccelZ;

  private static class AcquireTask implements Runnable {
    private ADIS16470_INS imu;

    public AcquireTask(ADIS16470_INS imu) {
      this.imu = imu;
    }

    @Override
    public void run() {
      imu.acquire();
    }
  }

  public ADIS16470_INS() {
    this(IMUAxis.X_CW, IMUAxis.Y_CW, IMUAxis.Z_CW, SPI.Port.kOnboardCS0, CalibrationTime._4s);
  }

  public ADIS16470_INS(IMUAxis pitchAxis, IMUAxis rollAxis, IMUAxis yawAxis) {
    this(pitchAxis, rollAxis, yawAxis, SPI.Port.kOnboardCS0, CalibrationTime._4s);
  }

  /**
   * @param yawAxis The axis that measures the yaw
   * @param port The SPI Port the gyro is plugged into
   * @param calibrationTime Calibration time
   */
  public ADIS16470_INS(IMUAxis pitchAxis, IMUAxis rollAxis, IMUAxis yawAxis, SPI.Port port, CalibrationTime calibrationTime) {
    robotPitch = pitchAxis;
    robotRoll  = rollAxis;
    robotYaw   = yawAxis;
    this.calibrationTime = calibrationTime;
    spiPort = port;

    m_acquire_task = new Thread(new AcquireTask(this));

    simDevice = SimDevice.create("Gyro:ADIS16470", port.value);
    if (simDevice != null) {
      m_simConnected = simDevice.createBoolean("connected", SimDevice.Direction.kInput, true);
      m_simGyroAngleX = simDevice.createDouble("gyro_angle_x", SimDevice.Direction.kInput, 0.0);
      m_simGyroAngleY = simDevice.createDouble("gyro_angle_y", SimDevice.Direction.kInput, 0.0);
      m_simGyroAngleZ = simDevice.createDouble("gyro_angle_z", SimDevice.Direction.kInput, 0.0);
      m_simGyroRateX = simDevice.createDouble("gyro_rate_x", SimDevice.Direction.kInput, 0.0);
      m_simGyroRateY = simDevice.createDouble("gyro_rate_y", SimDevice.Direction.kInput, 0.0);
      m_simGyroRateZ = simDevice.createDouble("gyro_rate_z", SimDevice.Direction.kInput, 0.0);
      m_simAccelX = simDevice.createDouble("accel_x", SimDevice.Direction.kInput, 0.0);
      m_simAccelY = simDevice.createDouble("accel_y", SimDevice.Direction.kInput, 0.0);
      m_simAccelZ = simDevice.createDouble("accel_z", SimDevice.Direction.kInput, 0.0);
    } else {
      // Force the IMU reset pin to toggle on startup (doesn't require DS enable)
      // Relies on the RIO hardware by default configuring an output as low
      // and configuring an input as high Z. The 10k pull-up resistor internal to the
      // IMU then forces the reset line high for normal operation.
      DigitalOutput tmpOutput = new DigitalOutput(27); // Drive SPI CS2 (IMU RST) low
      Timer.delay(0.01); // Wait 10ms
      tmpOutput.close();
      resetInput = new DigitalInput(27); // Set SPI CS2 (IMU RST) high
      Timer.delay(0.25); // Wait 250ms for reset to complete

      if (!switchToStandardSPI()) {
        return;
      }

      // Set IMU internal decimation to 4 (output data rate of 2000 SPS / (4 + 1) =
      // 400Hz)
      writeRegister(DEC_RATE, 4);

      // Set data ready polarity (HIGH = Good Data), Disable gSense Compensation and
      // PoP
      writeRegister(MSC_CTRL, 1);

      // Configure IMU internal Bartlett filter
      writeRegister(FILT_CTRL, 0);

      // Configure continuous bias calibration time based on user setting
      writeRegister(NULL_CNFG, (this.calibrationTime.value | 0x0700));

      // Notify DS that IMU calibration delay is active
      DriverStation.reportWarning("ADIS16470 IMU Detected. Starting initial calibration delay.", false);

      // Wait for samples to accumulate internal to the IMU (110% of user-defined
      // time)
      ThreadSleep((int)(Math.pow(2.0, this.calibrationTime.value) / 2000 * 64 * 1.1 * 1000));

      // Write offset calibration command to IMU
      writeRegister(GLOB_CMD, 0x0001);

      // Configure and enable auto SPI
      if (!switchToAutoSPI()) {
        return;
      }

      // Let the user know the IMU was initiallized successfully
      DriverStation.reportWarning("ADIS16470 IMU Successfully Initialized!", false);

      // Drive "Ready" LED low
      IMUStatusLed = new DigitalOutput(28); // Set SPI CS3 (IMU Ready LED) low
    }

    // Report usage and post data to DS
    HAL.report(tResourceType.kResourceType_ADIS16470, 0);
    IMUConnected = true;
  }

  public boolean isConnected() {
    if (m_simConnected != null) {
      return m_simConnected.get();
    }
    return IMUConnected;
  }

  /**
   * @param buf
   * @return buf's first 2 [0,1] bytes as a unsigned short stored in an int
   */
  private static int toUShort(ByteBuffer buf) {
    return (buf.getShort(0)) & 0xFFFF;
  }

  /**
   * @param sint signed int
   * @return sint as unsigned long with leading zeros
   */
  private static long toULong(int sint) {
    return sint & 0x00000000FFFFFFFFL;
  }

  /**
   * @param buf
   * @return buf's first 2 [0,1] bytes as signed short stored in a int
   */
  private static int toShort(int... buf) {
    return (short) (((buf[0] & 0xFF) << 8) + ((buf[1] & 0xFF) << 0));
  }

  /**
   * @param buf
   * @return buf's first 4 [0,1,2,3] bytes as signed int
   */
  private static int toInt(int... buf) {
    return (buf[0] & 0xFF) << 24 | (buf[1] & 0xFF) << 16 | (buf[2] & 0xFF) << 8 | (buf[3] & 0xFF);
  }

  private static int readBuffInt(int offset, int[] buff){
    return toInt(buff[offset], buff[offset + 1], buff[offset + 2], buff[offset + 3]);
  }

  private static int readBuffShort(int offset, int[] buff){
    return toShort(buff[offset], buff[offset + 1]);
  }

  private void ThreadSleep(int ms){
    try{
      Thread.sleep(ms);
    } catch (InterruptedException e){
    }
  }

  /**
   * Switch to standard SPI mode.
   *
   * @return
   */
  private boolean switchToStandardSPI() {
    // Check to see whether the acquire thread is active. If so, wait for it to stop
    // producing data.
    if (m_thread_active) {
      m_thread_active = false;
      while (!m_thread_idle) {
        ThreadSleep(10);
      }
      System.out.println("Paused the IMU processing thread successfully!");
      // Maybe we're in auto SPI mode? If so, kill auto SPI, and then SPI.
      if (spi != null && m_auto_configured) {
        stopAutoSPI();
      }
    }
    // There doesn't seem to be a SPI port active. Let's try to set one up
    if (spi == null) {
      System.out.println("Setting up a new SPI port.");
      spi = new SPI(spiPort);
      spi.setClockRate(2000000);//2,000,000 hz
      spi.setMode(SPI.Mode.kMode3);
      spi.setChipSelectActiveLow();

      // Validate the product ID
      readRegister(PROD_ID); // Dummy read
      if (readRegister(PROD_ID) != 16982) {
        DriverStation.reportError("Could not find ADIS16470", false);
        close();
        return false;
      }
      return true;
    } else {
      // Maybe the SPI port is active, but not in auto SPI mode? Try to read the
      // product ID.
      readRegister(PROD_ID); // dummy read
      if (readRegister(PROD_ID) != 16982) {
        DriverStation.reportError("Could not find an ADIS16470", false);
        close();
        return false;
      } else {
        return true;
      }
    }
  }
  
  private void stopAutoSPI(){
    spi.stopAuto();
    // We need to get rid of all the garbage left in the auto SPI buffer after
    // stopping it.
    // Sometimes data magically reappears, so we have to check the buffer size a
    // couple of times
    // to be sure we got it all. Yuck.
    int[] trashBuffer = new int[200];
    ThreadSleep(100);
    int data_count = spi.readAutoReceivedData(trashBuffer, 0, 0);
    while (data_count > 0) {
      spi.readAutoReceivedData(trashBuffer, Math.min(data_count, 200), 0);
      data_count = spi.readAutoReceivedData(trashBuffer, 0, 0);
    }
    System.out.println("Paused auto SPI successfully.");
  }

  /**
   * @return
   */
  boolean switchToAutoSPI() {
    // No SPI port has been set up. Go set one up first.
    if (spi == null && !switchToStandardSPI()) {
      DriverStation.reportError("Failed to start/restart auto SPI", false);
      return false;
    }
    // Only set up the interrupt if needed.
    if (m_auto_interrupt == null) {
      // Configure interrupt on SPI CS1
      m_auto_interrupt = new DigitalInput(26);
    }
    // The auto SPI controller gets angry if you try to set up two instances on one bus.
    if (!m_auto_configured) {
      spi.initAuto(8200);
      m_auto_configured = true;
    }
    // set spi packet (what is wanted back from imu)
    spi.setAutoTransmitData(autoSPIPacket, 2);
    // Configure auto stall time
    spi.configureAutoStall(5, 1000, 1);
    // Kick off auto SPI (Note: Device configuration impossible after auto SPI is
    // activated)
    // DR High = Data good (data capture should be triggered on the rising edge)
    spi.startAutoTrigger(m_auto_interrupt, true, false);

    // Check to see if the acquire thread is running. If not, kick one off.
    m_thread_active = true;
    m_first_run = true;
    if (!m_acquire_task.isAlive()) { m_acquire_task.start(); }
    System.out.println("Processing thread activated!");
    
    // Looks like the thread didn't start for some reason. Abort.
    if (!m_acquire_task.isAlive()) {
      DriverStation.reportError("Failed to start/restart the acquire() thread.", false);
      close();
      return false;
    }
    return true;
  }

  /**
   * Configures calibration time
   *
   * @param new_cal_time New calibration time
   * @return 1 if the new calibration time is the same as the current one, 2 error, 0 success
   */
  public int configCalTime(CalibrationTime new_cal_time) {
    if (calibrationTime.equals(new_cal_time))
      return 1;
    if (!switchToStandardSPI()) {
      DriverStation.reportError("Failed to configure/reconfigure standard SPI.", false);
      return 2;
    }
    calibrationTime = new_cal_time;
    // Configure continuous bias calibration time based on user setting
    writeRegister(NULL_CNFG, (calibrationTime.value | 0x700));
    if (!switchToAutoSPI()) {
      DriverStation.reportError("Failed to configure/reconfigure auto SPI.", false);
      return 2;
    }
    return 0;
  }

  /** TODO update to better implimentation */
  public int configDecRate(int reg) {
    int m_reg = reg;
    if (!switchToStandardSPI()) {
      DriverStation.reportError("Failed to configure/reconfigure standard SPI.", false);
      return 2;
    }
    if (m_reg > 1999) {
      DriverStation.reportError("Attempted to write an invalid deimation value.", false);
      m_reg = 1999;
    }
    m_scaled_sample_rate = (((m_reg + 1.0) / 2000.0) * 1000000.0);
    writeRegister(DEC_RATE, m_reg);
    System.out.println("Decimation register: " + readRegister(DEC_RATE));
    if (!switchToAutoSPI()) {
      DriverStation.reportError("Failed to configure/reconfigure auto SPI.", false);
      return 2;
    }
    return 0;
  }

  /**
   * Calibrate the gyro. It's important to make sure that the robot is not moving while the
   * calibration is in progress, this is typically done when the robot is first turned on while it's
   * sitting at rest before the match starts.
   */
  public void calibrate() {
    if (!switchToStandardSPI()) {
      DriverStation.reportError("Failed to configure/reconfigure standard SPI.", false);
    }
    writeRegister(GLOB_CMD, 0x0001);
    if (!switchToAutoSPI()) {
      DriverStation.reportError("Failed to configure/reconfigure auto SPI.", false);
    }
  }

  /**
   * Sets the robot axis retive to the imu axes
   *
   * @param yawAxis The new yaw axis to use
   * @return 1 if the new yaw axis is the same as the current one, 2 error, else 0.
   */
  public int setAxes(IMUAxis pitchAxis, IMUAxis rollAxis, IMUAxis yawAxis) {
    if (robotPitch == pitchAxis && robotRoll == rollAxis && robotYaw == yawAxis) {
      return 1;
    }
    if(yawAxis.equal(pitchAxis) || yawAxis.equal(rollAxis) || pitchAxis.equal(rollAxis)){
      DriverStation.reportError("Cant set multiple Robot axes to the same IMU axis.", true);
      return 2;
    }
    if (!switchToStandardSPI()) {
      DriverStation.reportError("Failed to configure/reconfigure standard SPI.", false);
      return 2;
    }
    robotPitch = pitchAxis;
    robotRoll = rollAxis;
    robotYaw = yawAxis;
    if (!switchToAutoSPI()) {
      DriverStation.reportError("Failed to configure/reconfigure auto SPI.", false);
    }
    return 0;
  }

  /**
   * @param reg
   * @return
   */
  private int readRegister(int reg) {
    ByteBuffer buf = ByteBuffer.allocateDirect(2);
    buf.order(ByteOrder.BIG_ENDIAN);
    buf.put(0, (byte) (reg & 0x7f));
    buf.put(1, (byte) 0);

    spi.write(buf, 2);
    spi.read(false, buf, 2);

    return toUShort(buf);
  }

  /**
   * @param reg
   * @param val
   */
  private void writeRegister(int reg, int val) {
    ByteBuffer buf = ByteBuffer.allocateDirect(2);
    // low byte
    buf.put(0, (byte) ((0x80 | reg)));
    buf.put(1, (byte) (val & 0xff));
    spi.write(buf, 2);
    // high byte
    buf.put(0, (byte) (0x81 | reg));
    buf.put(1, (byte) (val >> 8));
    spi.write(buf, 2);
  }

  public synchronized void reset() {
    FieldAngularPosX = 0.0;
    FieldAngularPosY = 0.0;
    FieldAngularPosZ = 0.0;
    FieldLinearPosX  = 0.0;
    FieldLinearPosY  = 0.0;
    FieldLinearPosZ  = 0.0;
  }

  public synchronized void setPos(Pose3d newPos){
    Rotation3d rot = newPos.getRotation();
    FieldAngularPosX = rot.getX();
    FieldAngularPosY = rot.getY();
    FieldAngularPosZ = rot.getZ();
    FieldLinearPosX  = newPos.getX();
    FieldLinearPosY  = newPos.getY();
    FieldLinearPosZ  = newPos.getZ();
  }

  /** Delete (free) the spi port used for the IMU. */
  @Override
  public void close() {
    if (m_thread_active) {
      m_thread_active = false;
      try {
        if (m_acquire_task != null) {
          m_acquire_task.join();
          m_acquire_task = null;
        }
      } catch (InterruptedException e) {}
      if (spi != null) {
        if (m_auto_configured) {
          spi.stopAuto();
        }
        spi.close();
        m_auto_configured = false;
        if (m_auto_interrupt != null) {
          m_auto_interrupt.close();
          m_auto_interrupt = null;
        }
        spi = null;
      }
    }
    System.out.println("Finished cleaning up after the IMU driver.");
  }

  /** */
  private void acquire() {
    // Set data packet length
    final int dataset_len = 35; // 12 d_gyro, 12 d_accel, 6 accel, 2 error, 2 junk?, 1 timestamp
    final int BUFFER_SIZE = 8000;
    // effective size of the buffer if it is to hold the max number of whole datasets
    final int BUFFER_SIZE_EFFECTIVE = BUFFER_SIZE - (BUFFER_SIZE % dataset_len);

    // Set up buffers and variables
    int[]  buffer = new int[BUFFER_SIZE];
    int    data_to_read = 0;
    double previous_timestamp = 0.0;
    double elapsedSamples = 0.0;
    double[] robotDeltaAng = {0.0,0.0,0.0};
    double[] robotDeltaVel = {0.0,0.0,0.0};
    double[] robotAccel    = {0.0,0.0,0.0};

    while (true) {
      // Sleep loop for 10ms
      ThreadSleep(1000);



      if (m_thread_active) {
        m_thread_idle = false;

        // Count number of full packets waiting in the DMA buffer
        data_to_read = countDMABufferElements(dataset_len, BUFFER_SIZE_EFFECTIVE);
        
        // Read data from DMA buffer (only complete sets)
        //  0 timestamp,    1 idk,          2 idk, 
        //  3 , 4 , 5 , 6 , 
        //  7 AngAccelX_H,  8 AngAccelX_L,  9 AngAccelY_H, 10 AngAccelY_L, 11 AngAccelZ_H, 12 AngAccelZ_L,
        // 13 AccelX_H,    14 AccelX_L,    15 AccelY_H,    16 AccelY_L,    17 AccelZ_H,    18 AccelZ_L,
        spi.readAutoReceivedData(buffer, data_to_read, 0);

        // Could be multiple data sets in the buffer. Handle each one.
        for (int i = 0; i < Math.min(data_to_read,1); i += dataset_len) {
          // Timestamp is at buffer[i]
          m_dt = ((double) buffer[i] - previous_timestamp) / 1000000.0;

          /*
           * Get delta angle value for selected yaw axis and scale by the elapsed time
           * (based on timestamp)
           */
          // samples covered by the delta reading
          elapsedSamples = m_scaled_sample_rate / (buffer[i] - previous_timestamp);
          System.out.println("---------------------");
          robotDeltaAng = bufferReadAng(  buffer, i + 3,  elapsedSamples);
          System.out.printf("%f, %f, %f - ",robotDeltaAng[0], robotDeltaAng[1],robotDeltaAng[2]);
          robotDeltaVel = bufferReadVel(  buffer, i + 15, elapsedSamples);
          System.out.printf("%f, %f, %f - ",robotDeltaVel[0], robotDeltaVel[1],robotDeltaVel[2]);
          robotAccel    = bufferReadAccel(buffer, i + 27, elapsedSamples);
          System.out.printf("%f, %f, %f - ",robotAccel[0], robotAccel[1],robotAccel[2]);
          IMUDebugOut = buffer[i + 33];
          System.out.printf("%s\n",Integer.toString(IMUDebugOut, 2));

          // Store timestamp for next iteration
          previous_timestamp = buffer[i];

          m_first_run = false;
        }
      } else {
        m_thread_idle = true;
        previous_timestamp = 0.0;
      }
    }
  }

  /** subfunction of aquire. counts number of full packets waiting in the DMA buffer */
  private int countDMABufferElements(int DATASET_LEN, int BUFFER_SIZE_EFFECTIVE){
    // Read number of bytes currently waiting in the DMA buffer
    int data_count = spi.readAutoReceivedData(new int[1], 0, 0); 

    // reduce to only full frames of data
    int data_to_read = data_count - (data_count % DATASET_LEN);

    // Want to cap the data to read in a single read at the buffer size
    if (data_to_read > BUFFER_SIZE_EFFECTIVE) {
      DriverStation.reportWarning("ADIS16470 data processing thread overrun has occurred!", false);
      data_to_read = BUFFER_SIZE_EFFECTIVE;
    }
    return data_to_read;
  }
  
  /** subfunction of aquire. Reads angle change out of the buffer array
   * 
   * @param buffer array read from DMA buffer
   * @param i index into the array
   * @param elapsedSamples 
   * @return
   */
  private double[] bufferReadAng(int[] buffer, int i, double elapsedSamples){
    double[] IMUDeltaAngle = {0.0,0.0,0.0};
    IMUDeltaAngle[robotPitch.axis() - 1] = 
        (readBuffInt(i,     buffer) * MULT_DELTA_ANGLE_SF_32) / elapsedSamples * robotPitch.direction();
    IMUDeltaAngle[robotRoll.axis()  - 1] = 
        (readBuffInt(i + 4, buffer) * MULT_DELTA_ANGLE_SF_32) / elapsedSamples * robotRoll.direction();
    IMUDeltaAngle[robotYaw.axis()   - 1] = 
        (readBuffInt(i + 8, buffer) * MULT_DELTA_ANGLE_SF_32) / elapsedSamples * robotYaw.direction();
    return IMUDeltaAngle;
  }
  
  /** subfunction of aquire. Reads angle change out of the buffer array
   * 
   * @param buffer array read from DMA buffer
   * @param i index into the array
   * @param elapsedSamples 
   * @return
   */
  private double[] bufferReadVel(int[] buffer, int i, double elapsedSamples){
    double[] IMUDeltaVel = {0.0,0.0,0.0};
    IMUDeltaVel[robotPitch.axis() - 1] = 
        (readBuffInt(i,     buffer) * MULT_DELTA_VEL_SF_32) / elapsedSamples * robotPitch.direction();
    IMUDeltaVel[robotRoll.axis()  - 1] =
        (readBuffInt(i + 4, buffer) * MULT_DELTA_VEL_SF_32) / elapsedSamples * robotRoll.direction();
    IMUDeltaVel[robotYaw.axis()   - 1] = 
        (readBuffInt(i + 8, buffer) * MULT_DELTA_VEL_SF_32) / elapsedSamples * robotYaw.direction();
    return IMUDeltaVel;
  }
  
  /** subfunction of aquire. Reads angle change out of the buffer array
   * 
   * @param buffer array read from DMA buffer
   * @param i index into the array
   * @param elapsedSamples 
   * @return
   */
  private double[] bufferReadAccel(int[] buffer, int i, double elapsedSamples){
    double[] IMUAccel = {0.0,0.0,0.0};
    IMUAccel[robotPitch.axis() - 1] = 
        (readBuffShort(i,     buffer) * MULT_ACCEL_SF_16) * robotPitch.direction();
    IMUAccel[robotRoll.axis()  - 1] = 
        (readBuffShort(i + 2, buffer) * MULT_ACCEL_SF_16) * robotRoll.direction();
    IMUAccel[robotYaw.axis()   - 1] = 
        (readBuffShort(i + 4, buffer) * MULT_ACCEL_SF_16) * robotYaw.direction();
    return IMUAccel;
  }

  /**
   * @param compAngle
   * @param accAngle
   * @return
   */
  private double formatFastConverge(double compAngle, double accAngle) {
    if (compAngle > accAngle + Math.PI) {
      compAngle = compAngle - 2.0 * Math.PI;
    } else if (accAngle > compAngle + Math.PI) {
      compAngle = compAngle + 2.0 * Math.PI;
    }
    return compAngle;
  }

  /** changes compAngle from continous -Inf to Inf continuing to count up or down as the angle completes a circle
   *  to the discontinous range 0 to 2*pi jumping as the angle reaches either end
   * @param compAngle
   * @return
   */
  private double formatRange0to2PI(double compAngle) {
    return (compAngle < 0 ? 2*Math.PI : 0) + (compAngle % (2*Math.PI));
  }

  /**
   * @param accelAngle
   * @param accelZ
   * @return
   */
  private double formatAccelRange(double accelAngle, double accelZ) {
    if (accelZ < 0.0) {
      accelAngle = Math.PI - accelAngle;
    } else if (accelZ > 0.0 && accelAngle < 0.0) {
      accelAngle = 2.0 * Math.PI + accelAngle;
    }
    return accelAngle;
  }

  /**
   * @param compAngle
   * @param accelAngle
   * @param omega
   * @return
   */
  private double compFilterProcess(double compAngle, double accelAngle, double omega) {
    compAngle = formatFastConverge(compAngle, accelAngle);
    // compAngle = m_alpha * (compAngle + omega * m_dt) + (1.0 - m_alpha) * accelAngle;
    compAngle = formatRange0to2PI(compAngle);
    if (compAngle > Math.PI) {
      compAngle = compAngle - 2.0 * Math.PI;
    }
    return compAngle;
  }

  public synchronized double getAngle(){return getAngle(robotYaw);}
  /**
   * @return Yaw axis angle in degrees (CCW positive)
   */
  public synchronized double getAngle(IMUAxis robotAxis) {
    switch (robotAxis) {
      case X_CW:
        if (m_simGyroAngleX != null) {
          return m_simGyroAngleX.get();
        }
        break;
      case X_ACW:
        if (m_simGyroAngleX != null) {
          return -m_simGyroAngleX.get();
        }
        break;
      case Y_CW:
        if (m_simGyroAngleY != null) {
          return m_simGyroAngleY.get();
        }
        break;
      case Y_ACW:
        if (m_simGyroAngleY != null) {
          return -m_simGyroAngleY.get();
        }
        break;
      case Z_CW:
        if (m_simGyroAngleZ != null) {
          return m_simGyroAngleZ.get();
        }
        break;
      case Z_ACW:
        if (m_simGyroAngleZ != null) {
          return -m_simGyroAngleZ.get();
        }
        break;
    }
    return 1 ;//m_integ_angle;
  }

  /**
   * @return Yaw axis angular rate in degrees per second (CCW positive)
   */
  public synchronized double getRate() {
    if (robotYaw == IMUAxis.X_CW) {
      if (m_simGyroRateX != null) {
        return m_simGyroRateX.get();
      }
      return 1;//IMUAngularVelX;
    } else if (robotYaw == IMUAxis.Y_CW) {
      if (m_simGyroRateY != null) {
        return m_simGyroRateY.get();
      }
      return 1;//IMUAngularVelY;
    } else if (robotYaw == IMUAxis.Z_CW) {
      if (m_simGyroRateZ != null) {
        return m_simGyroRateZ.get();
      }
      return 1;//IMUAngularVelZ;
    } else {
      return 0.0;
    }
  }

  /**
   * @return Yaw Axis
   */
  public IMUAxis getPitchAxis() {
    return robotPitch;
  }
  /**
   * @return Yaw Axis
   */
  public IMUAxis getRollAxis() {
    return robotRoll;
  }
  /**
   * @return Yaw Axis
   */
  public IMUAxis getYawAxis() {
    return robotYaw;
  }

  /**
   * @return current acceleration in the X axis
   */
  public synchronized double getAccelX() {
    return 1;//IMULinearAccelX * localGravity;
  }

  /**
   * @return current acceleration in the Y axis
   */
  public synchronized double getAccelY() {
    return 1;//IMULinearAccelY * localGravity;
  }

  /**
   * @return current acceleration in the Z axis
   */
  public synchronized double getAccelZ() {
    return 1;//IMULinearAccelZ * localGravity;
  }

  /**
   * Get the SPI port number.
   *
   * @return The SPI port number.
   */
  public int getPort() {
    return spiPort.value;
  }

  @Override
  public void initSendable(NTSendableBuilder builder) {
    builder.setSmartDashboardType("Gyro");
    builder.addDoubleProperty("Value", this::getAngle, null);
  }
}

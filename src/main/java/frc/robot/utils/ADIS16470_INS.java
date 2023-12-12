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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
  //"unused",
  "PMD.RedundantFieldInitializer",
  "PMD.ImmutableField",
  "PMD.SingularField",
  "PMD.CollapsibleIfStatements",
  "PMD.MissingOverride",
  "PMD.EmptyIfStmt",
  "PMD.EmptyStatementNotInLoop"
})
public class ADIS16470_INS implements AutoCloseable, NTSendable {
  /** ADIS16470 Register Map Declaration */
  private static enum RegMap{
    FLASH_CNT(0x00), // Flash memory write count
    //private static final int FLASH_CNT = 0x00; 
    DIAG_STAT(0x02), // Diagnostic and operational status
    // 7 bit - Clock Sync Error (clock has not synced with external clock)
    // 6 bit - Memory Failure (memory checksum error)(hope you dont see this)
    // 5 bit - Sensor Failure (1+ gyro/accel fail internal test)
    // 4 bit - Low Power Standby (supply voltage too low)(you shouldn't see this)
    // 3 bit - SPI Comms Error (error in the comms system behind the sceens)
    // 2 bit - Flash Memory Update Failure (prod wont see this)
    // 1 bit - Data Path Overrun (something internal overran restart the imu and go again)

    X_GYRO_LOW(0x04), // X-axis gyroscope output, lower word
    X_GYRO_OUT(0x06), // X-axis gyroscope output, upper word
    Y_GYRO_LOW(0x08), // Y-axis gyroscope output, lower word
    Y_GYRO_OUT(0x0A), // Y-axis gyroscope output, upper word
    Z_GYRO_LOW(0x0C), // Z-axis gyroscope output, lower word
    Z_GYRO_OUT(0x0E), // Z-axis gyroscope output, upper word
    X_ACCL_LOW(0x10), // X-axis accelerometer output, lower word
    X_ACCL_OUT(0x12), // X-axis accelerometer output, upper word
    Y_ACCL_LOW(0x14), // Y-axis accelerometer output, lower word
    Y_ACCL_OUT(0x16), // Y-axis accelerometer output, upper word
    Z_ACCL_LOW(0x18), // Z-axis accelerometer output, lower word
    Z_ACCL_OUT(0x1A), // Z-axis accelerometer output, upper word
    TEMP_OUT(0x1C), // Temperature output (internal, not calibrated)
    TIME_STAMP(0x1E), // PPS mode time stamp
    X_DELTANG_LOW(0x24), // X-axis delta angle output, lower word
    X_DELTANG_OUT(0x26), // X-axis delta angle output, upper word
    Y_DELTANG_LOW(0x28), // Y-axis delta angle output, lower word
    Y_DELTANG_OUT(0x2A), // Y-axis delta angle output, upper word
    Z_DELTANG_LOW(0x2C), // Z-axis delta angle output, lower word
    Z_DELTANG_OUT(0x2E), // Z-axis delta angle output, upper word
    X_DELTVEL_LOW(0x30), // X-axis delta velocity output, lower word
    X_DELTVEL_OUT(0x32), // X-axis delta velocity output, upper word
    Y_DELTVEL_LOW(0x34), // Y-axis delta velocity output, lower word
    Y_DELTVEL_OUT(0x36), // Y-axis delta velocity output, upper word
    Z_DELTVEL_LOW(0x38), // Z-axis delta velocity output, lower word
    Z_DELTVEL_OUT(0x3A), // Z-axis delta velocity output, upper word
    XG_BIAS_LOW( 0x40), // X-axis gyroscope bias offset correction, lower word
    XG_BIAS_HIGH(0x42), // X-axis gyroscope bias offset correction, upper word
    YG_BIAS_LOW( 0x44), // Y-axis gyroscope bias offset correction, lower word
    YG_BIAS_HIGH(0x46), // Y-axis gyroscope bias offset correction, upper word
    ZG_BIAS_LOW( 0x48), // Z-axis gyroscope bias offset correction, lower word
    ZG_BIAS_HIGH(0x4A), // Z-axis gyroscope bias offset correction, upper word
    XA_BIAS_LOW( 0x4C), // X-axis accelerometer bias offset correction, lower word
    XA_BIAS_HIGH(0x4E), // X-axis accelerometer bias offset correction, upper word
    YA_BIAS_LOW( 0x50), // Y-axis accelerometer bias offset correction, lower word
    YA_BIAS_HIGH(0x52), // Y-axis accelerometer bias offset correction, upper word
    ZA_BIAS_LOW( 0x54), // Z-axis accelerometer bias offset correction, lower word
    ZA_BIAS_HIGH(0x56), // Z-axis accelerometer bias offset correction, upper word
    FILT_CTRL(0x5C), // Filter control
    MSC_CTRL(0x60), // Miscellaneous control
    UP_SCALE(0x62), // Clock scale factor, PPS mode
    DEC_RATE(0x64), // Decimation rate control (output data rate)
    NULL_CNFG(0x66), // Auto-null configuration control
    GLOB_CMD(0x68), // Global commands
    FIRM_REV(0x6C), // Firmware revision
    FIRM_DM(0x6E), // Firmware revision date, month and day
    FIRM_Y(0x70), // Firmware revision date, year
    PROD_ID(0x72), // Product identification
    SERIAL_NUM(0x74), // Serial number (relative to assembly lot)
    USER_SCR1(0x76), // User scratch register 1
    USER_SCR2(0x78), // User scratch register 2
    USER_SCR3(0x7A), // User scratch register 3
    FLSHCNT_LOW(0x7C), // Flash update count, lower word
    FLSHCNT_HIGH(0x7E); // Flash update count, upper word

    private final int label;
    public byte val() {return (byte)label;}
    private RegMap(int label) { this.label = label; }
  }

  protected byte[] autoSPIPacket = new byte[16];

  protected void makeSPIPacket(){
    byte[] bot_x_bytes = robotRoll.packetBytes();
    byte[] bot_y_bytes = robotPitch.packetBytes();
    byte[] bot_z_bytes = robotYaw.packetBytes();

    autoSPIPacket[0]  = bot_x_bytes[4]; //roll out
    autoSPIPacket[1]  = RegMap.FLASH_CNT.val();
    autoSPIPacket[2]  = bot_y_bytes[4]; //pitch out
    autoSPIPacket[3]  = RegMap.FLASH_CNT.val();
    autoSPIPacket[4]  = bot_z_bytes[4]; //yaw out
    autoSPIPacket[5]  = RegMap.FLASH_CNT.val();
    autoSPIPacket[6]  = bot_z_bytes[5]; //yaw low
    autoSPIPacket[7]  = RegMap.FLASH_CNT.val();

    autoSPIPacket[8]  = bot_x_bytes[6]; //forward/back out
    autoSPIPacket[9]  = RegMap.FLASH_CNT.val();
    autoSPIPacket[10] = bot_x_bytes[7]; //forward/back low
    autoSPIPacket[11] = RegMap.FLASH_CNT.val();
    autoSPIPacket[12] = bot_y_bytes[6]; //left/right out
    autoSPIPacket[13] = RegMap.FLASH_CNT.val();
    autoSPIPacket[14] = bot_z_bytes[6]; //up/down out
    autoSPIPacket[15] = RegMap.FLASH_CNT.val();
  }

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
    private CalibrationTime(int value) { this.value = value; }

    private boolean equals(CalibrationTime calTime){
      return value == calTime.value;
    }
  }

  /**
   * axis are built around the standard
   * pos = CW
   * CW Pitch = pitch up (lift robot front)
   * CW Roll = right roll (top of robot moves right)
   * CW Yaw = right turn (front of robot moves right)
   */
  public enum IMUAxis {
    X_CW(1),
    X_ACW(-1),
    Y_CW(2),
    Y_ACW(-2),
    Z_CW(3),
    Z_ACW(-3);

    private int value;
    private IMUAxis(int value){ this.value = value; }

    private boolean equal(IMUAxis cmp){
      return Math.abs(value) == Math.abs(cmp.value);
    }

    protected int direction(){
      return Integer.signum(value);
    }
    protected int axis(){
      return Math.abs(value);
    }
    protected byte[] packetBytes(){
      switch (axis()){
        case 1: return new byte[] {RegMap.X_GYRO_OUT.val(),     RegMap.X_GYRO_LOW.val(),      //gyro angle
                                   RegMap.X_ACCL_OUT.val(),     RegMap.X_ACCL_LOW.val(),      //linear acceleration
                                   RegMap.X_DELTANG_OUT.val(),  RegMap.X_DELTANG_LOW.val(),   //change in angle
                                   RegMap.X_DELTVEL_OUT.val(),  RegMap.X_DELTVEL_LOW.val()};  //change in speed

        case 2: return new byte[] {RegMap.Y_GYRO_OUT.val(),     RegMap.Y_GYRO_LOW.val(), 
                                   RegMap.Y_ACCL_OUT.val(),     RegMap.Y_ACCL_LOW.val(), 
                                   RegMap.Y_DELTANG_OUT.val(),  RegMap.Y_DELTANG_LOW.val(), 
                                   RegMap.Y_DELTVEL_OUT.val(),  RegMap.Y_DELTVEL_LOW.val()};
                                   
        case 3: return new byte[] {RegMap.Z_GYRO_OUT.val(),     RegMap.Z_GYRO_LOW.val(),   
                                   RegMap.Z_ACCL_OUT.val(),     RegMap.Z_ACCL_LOW.val(),   
                                   RegMap.Z_DELTANG_OUT.val(),  RegMap.Z_DELTANG_LOW.val(),
                                   RegMap.Z_DELTVEL_OUT.val(),  RegMap.Z_DELTVEL_LOW.val()};
      }
      return new byte[] {0,0, 0,0, 0,0, 0,0};
    }
  }

  // Static Constants
  //data sheet page 22 table 59, conversion from responce to number
  protected static final double MULT_GYRO_ANGLE_SF_16  = 0.1;
  protected static final double MULT_GYRO_ANGLE_SF_32  = 0.1 / 65536; /* 0.1 / 2^16 */
  protected static final double MULT_ACCEL_SF_16       = 0.00125; /* 0.125 / 1000 */
  protected static final double MULT_ACCEL_SF_32       = 0.00125 / 65536; /* 0.125 / 1000 / 2^16 */
  protected static final double MULT_DELTA_ANGLE_SF_16 = 2160.0 / 2147483648.0; /* 2160 / (2^31) */
  protected static final double MULT_DELTA_ANGLE_SF_32 = 2160.0 / 2147483648.0; /* 2160 / (2^31) */
  protected static final double MULT_DELTA_VEL_SF_16   = 400.0 / 2147483648.0; /*  400 / (2^31) */
  protected static final double MULT_DELTA_VEL_SF_32   = 400.0 / 2147483648.0; /*  400 / (2^31) */
  protected static double localGravity = 9.81;

  // calculation variables
  protected double timeStep = 0.0;

  // User-specified IMU axes
  protected IMUAxis robotPitch;
  protected IMUAxis robotRoll;
  protected IMUAxis robotYaw;

  /** positionof the robot relitive to some origin in mm and degrees */
  protected Pose3d      RobotLocation = new Pose3d();
  /** linear and angular change over the last timestep not over the last second */
  protected Transform3d RobotDelta = new Transform3d();
  /** linear and angular acceleration over the last timestep not second */
  protected Transform3d RobotDelta2 = new Transform3d();

  // State variables
  private volatile boolean m_thread_active = false;
  private CalibrationTime calibrationTime;
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
      imu.readIMU();
    }
  }

  public ADIS16470_INS() {
    this(IMUAxis.X_CW, IMUAxis.Y_CW, IMUAxis.Z_CW, SPI.Port.kOnboardCS0, CalibrationTime._8s);
  }

  public ADIS16470_INS(IMUAxis rollAxis, IMUAxis pitchAxis, IMUAxis yawAxis) {
    this(rollAxis, pitchAxis, yawAxis, SPI.Port.kOnboardCS0, CalibrationTime._8s);
  }

  /**
   * @param yawAxis The axis that measures the yaw
   * @param port The SPI Port the gyro is plugged into
   * @param calibrationTime Calibration time
   */
  public ADIS16470_INS(IMUAxis rollAxis, IMUAxis pitchAxis, IMUAxis yawAxis, SPI.Port port, CalibrationTime calibrationTime) {
    
    changeAxes(rollAxis, pitchAxis, yawAxis);

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

      // Set IMU internal decimation to 4 (output data rate of 2000 SPS / (4 + 1) = 400Hz)
      writeRegister(RegMap.DEC_RATE.val(), 4);

      // Set data ready polarity (HIGH = Good Data), Disable gSense Compensation and PoP
      writeRegister(RegMap.MSC_CTRL.val(), 1);

      // Configure IMU internal Bartlett filter
      writeRegister(RegMap.FILT_CTRL.val(), 0);

      // Configure continuous bias calibration time based on user setting
      writeRegister(RegMap.NULL_CNFG.val(), (this.calibrationTime.value | 0x0700));

      // Notify DS that IMU calibration delay is active
      DriverStation.reportWarning("ADIS16470 IMU Detected. Starting initial calibration delay.", false);

      // Wait for samples to accumulate internal to the IMU (110% of user-defined
      // time)
      ThreadSleep((int)(Math.pow(2.0, this.calibrationTime.value) / 2000 * 64 * 1.1 * 1000));

      // Write offset calibration command to IMU
      writeRegister(RegMap.GLOB_CMD.val(), 0x0001);

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
      readRegister(RegMap.PROD_ID.val()); // Dummy read
      if (readRegister(RegMap.PROD_ID.val()) != 16982) {
        DriverStation.reportError("Could not find ADIS16470", false);
        close();
        return false;
      }
      return true;
    } else {
      // Maybe the SPI port is active, but not in auto SPI mode? Try to read the
      // product ID.
      readRegister(RegMap.PROD_ID.val()); // dummy read
      if (readRegister(RegMap.PROD_ID.val()) != 16982) {
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
    writeRegister(RegMap.NULL_CNFG.val(), (calibrationTime.value | 0x700));
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
    writeRegister(RegMap.DEC_RATE.val(), m_reg);
    System.out.println("Decimation register: " + readRegister(RegMap.DEC_RATE.val()));
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
    writeRegister(RegMap.GLOB_CMD.val(), 0x0001);
    if (!switchToAutoSPI()) {
      DriverStation.reportError("Failed to configure/reconfigure auto SPI.", false);
    }
  }

  /**
   * Sets the robot axis relitive to the imu axes
   *
   * @param yawAxis The new yaw axis to use
   * @return 1 if the new yaw axis is the same as the current one, 2 error, else 0.
   */
  public int setAxes(IMUAxis rollAxis, IMUAxis pitchAxis, IMUAxis yawAxis) {
    if (!switchToStandardSPI()) {
      DriverStation.reportError("Failed to configure/reconfigure standard SPI.", false);
      return 2;
    }

    int tmp;
    if((tmp = changeAxes(rollAxis, pitchAxis, yawAxis)) != 0)
        return tmp;

    if (!switchToAutoSPI()) {
      DriverStation.reportError("Failed to configure/reconfigure auto SPI.", false);
    }
    return 0;
  }
  private int changeAxes(IMUAxis rollAxis, IMUAxis pitchAxis, IMUAxis yawAxis) {
    if (robotRoll == rollAxis && robotPitch == pitchAxis && robotYaw == yawAxis) {
      return 1;
    }
    if(yawAxis.equal(pitchAxis) || yawAxis.equal(rollAxis) || pitchAxis.equal(rollAxis)){
      DriverStation.reportError("Cant set multiple Robot axes to the same IMU axis.", true);
      return 2;
    }
    
    robotRoll = rollAxis;
    robotPitch = pitchAxis;
    robotYaw = yawAxis;
    makeSPIPacket();
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

  /** reset position, velocity, and acceleration to 0 ensure robot is stopped*/
  public synchronized void reset() {
    RobotLocation = new Pose3d();
    RobotDelta    = new Transform3d();
    RobotDelta2   = new Transform3d();
  }

  /** change current location can occur while moving */
  public synchronized void setPos(Pose3d newPos){
    RobotLocation = newPos;
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
  private void readIMU() {
    // Set data packet length
    final int dataset_len = 19; // 16 data responce, 2 junk?, 1 timestamp
    final int BUFFER_SIZE = 4000;
    // effective size of the buffer if it is to hold the max number of whole datasets
    final int BUFFER_SIZE_EFFECTIVE = BUFFER_SIZE - (BUFFER_SIZE % dataset_len);

    // Set up buffers and variables
    int[]  buffer = new int[BUFFER_SIZE];
    double previous_timestamp = 0.0;
    double elapsedSamples = 0.0;
    Rotation3d robotDeltaAng;
    Translation3d robotDeltaVel;

    //clear initialization data
    int data_to_read = countDMABufferElements(dataset_len, BUFFER_SIZE_EFFECTIVE);
    spi.readAutoReceivedData(buffer, data_to_read, 0);

    while (true) {
      // Sleep loop for 10ms
      ThreadSleep(10);

      if (m_thread_active) {
        m_thread_idle = false;

        // Count number of full packets waiting in the DMA buffer
        data_to_read = countDMABufferElements(dataset_len, BUFFER_SIZE_EFFECTIVE);
        spi.readAutoReceivedData(buffer, data_to_read, 0);

        if(data_to_read > 0 && (buffer[0] < previous_timestamp || buffer[1] != 0 || buffer[2] != 0)){
          throw new IllegalStateException("IMU SPI Read desync");
        }

        // Could be multiple data sets in the buffer. Handle each one.
        for (int i = 0; i < data_to_read; i += dataset_len) {
          // Timestamp is at buffer[i]
          
          // Get delta angle value for selected yaw axis and scale by the elapsed time (based on timestamp)
          // samples covered by the delta reading
          elapsedSamples = m_scaled_sample_rate / (buffer[i] - previous_timestamp);
          
          robotDeltaAng = bufferReadAng(buffer, i + 3,  elapsedSamples);
          robotDeltaVel = bufferReadVel(buffer, i + 11, elapsedSamples);
          
          synchronized (this){
            timeStep = ((double) buffer[i] - previous_timestamp) / 1000000.0;
            RobotDelta2 = new Transform3d(robotDeltaVel.minus(RobotDelta.getTranslation()), 
                                          robotDeltaAng.minus(RobotDelta.getRotation()));
            RobotDelta = new Transform3d(robotDeltaVel, robotDeltaAng);
            RobotLocation = RobotLocation.plus(RobotDelta);
          }

          // Store timestamp for next iteration
          previous_timestamp = buffer[i];
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
   * @return [2byte pitch, 2 byte roll, 4byte yaw]
   */
  private Rotation3d bufferReadAng(int[] buffer, int i, double elapsedSamples){
    double d_roll = 
        (readBuffShort(i,     buffer) * MULT_DELTA_ANGLE_SF_16) / elapsedSamples * robotRoll.direction();
    double d_pitch = 
        (readBuffShort(i + 2, buffer) * MULT_DELTA_ANGLE_SF_16) / elapsedSamples * robotPitch.direction();
    double d_yaw = 
        (readBuffInt(  i + 4, buffer) * MULT_DELTA_ANGLE_SF_32) / elapsedSamples * robotYaw.direction();
    return new Rotation3d(Math.toRadians(d_roll), Math.toRadians(d_pitch), Math.toRadians(d_yaw));
  }
  
  /** subfunction of aquire. Reads angle change out of the buffer array
   * 
   * @param buffer array read from DMA buffer
   * @param i index into the array
   * @param elapsedSamples 
   * @return [X movement +R/-L, Y movement +F/-B, Z movement +U/-D]
   */
  private Translation3d bufferReadVel(int[] buffer, int i, double elapsedSamples){
    double dX = 
        (readBuffInt(  i,     buffer) * MULT_DELTA_VEL_SF_32) / elapsedSamples * robotRoll.direction();
    double dY =
        (readBuffShort(i + 4, buffer) * MULT_DELTA_VEL_SF_16) / elapsedSamples * robotPitch.direction();
    double dZ = 
        (readBuffShort(i + 6, buffer) * MULT_DELTA_VEL_SF_16) / elapsedSamples * robotYaw.direction();
    return new Translation3d(dX, dY, dZ);
  }

  public synchronized Pose2d getPose2d(){
    return RobotLocation.toPose2d();
  }
  public synchronized Pose3d getPose3d(){
    return RobotLocation;
  }

  public synchronized Transform3d getVelocity(){
    return RobotDelta.div(timeStep);
  }

  public synchronized Transform3d getAcceleration(){
    return RobotDelta2.div(timeStep);
  }

  public double getRoll(){ return getRoll(false); }
  public synchronized double getRoll(boolean radians){
    if(!radians)
      return Math.toDegrees(RobotLocation.getRotation().getX());
    return RobotLocation.getRotation().getX();
  }

  public double getPitch(){ return getPitch(false); }
  public synchronized double getPitch(boolean radians){
    if(!radians)
      return Math.toDegrees(RobotLocation.getRotation().getY());
    return RobotLocation.getRotation().getY();
  }

  public double getAngle() { return getYaw(false); }
  public double getYaw(){ return getYaw(false); }
  public synchronized double getYaw(boolean radians){
    if(!radians)
      return Math.toDegrees(RobotLocation.getRotation().getZ());
    return RobotLocation.getRotation().getZ();
  }
  
  @Override
  public synchronized String toString() {
    Transform3d robVel = getVelocity();
    Rotation3d robVelR = robVel.getRotation();
    double[] robVelRD = {Math.toDegrees(robVelR.getX()), Math.toDegrees(robVelR.getY()), 
                         Math.toDegrees(robVelR.getZ())};
    Transform3d robAcl = getAcceleration();
    Rotation3d robAclR = robAcl.getRotation();
    double[] robAclRD = {Math.toDegrees(robAclR.getX()), Math.toDegrees(robAclR.getY()), 
                         Math.toDegrees(robAclR.getZ())};

    return "IMU16470 INS : \n"+
            "  Pos {" +RobotLocation.getX()+", "+RobotLocation.getY()+", "+RobotLocation.getZ()+"} : "+
            "  Rot {" +getRoll(true)       +", "+getPitch(true)      +", "+getYaw(true)        +"} : \n"+
            "  LinV {"+robVel.getX()       +", "+robVel.getY()       +", "+robVel.getZ()       +"} : "+
            "  RotV {"+robVelRD[0]         +", "+robVelRD[1]         +", "+robVelRD[2]         +"} : \n"+
            "  LinA {"+robAcl.getX()       +", "+robAcl.getY()       +", "+robAcl.getZ()       +"} : "+
            "  RotA {"+robAclRD[0]         +", "+robAclRD[1]         +", "+robAclRD[2]         +"}";
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
    builder.addDoubleProperty("Value", this::getYaw, null);
  }
}

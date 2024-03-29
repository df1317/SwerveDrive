package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.CANCoderFaults;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveModule {
  int frameRateMs = 100; // Example: 100ms. Adjust this value based on your needs.
  private static final double kWheelRadius = 0.0508;
  private static final int kDriveEncoderResolution = 42;

  // change back to = Drivetrain.kMaxAngularSpeed
  private static final double kModuleMaxAngularVelocity = Math.PI * 2;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final CANCoder m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(1, 0, 0,
      new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   * We probably don't need drive encoder channels or turning encoder channel B,
   * so remove them in the season main code
   * 
   * @param driveMotorChannel      PWM output for the drive motor.
   * @param turningMotorChannel    PWM output for the turning motor.
   * @param driveEncoderChannelA   DIO input for the drive encoder channel A
   * @param driveEncoderChannelB   DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int canCoderChannel/*
                                                                     * , int driveEncoderChannelA, int
                                                                     * driveEncoderChannelB,
                                                                     * int turningEncoderChannelA, int
                                                                     * turningEncoderChannelB
                                                                     */) {

    // set up motors and encoders
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, frameRateMs);
    m_turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, frameRateMs);

    m_driveEncoder = m_driveMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, kDriveEncoderResolution);
    m_turningEncoder = new CANCoder(canCoderChannel);
    m_turningEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, frameRateMs, 50); // 50ms timeout

    CANCoderConfiguration config = new CANCoderConfiguration();
    // set units of the CANCoder to radians, with velocity being radians per second
    config.sensorCoefficient = 2 * Math.PI / 4096.0;
    config.unitString = "rad";
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    m_turningEncoder.configAllSettings(config);

    ErrorCode error = m_turningEncoder.getLastError();
    if (error != ErrorCode.OK) {
        DriverStation.reportError("Error initializing turning encoder: " + error, false);
    }

    CANCoderFaults faults = new CANCoderFaults();
    ErrorCode faultsError = m_turningEncoder.getFaults(faults);
    if (faultsError != ErrorCode.OK) {
        DriverStation.reportError("Error getting faults from turning encoder: " + faultsError, false);
    }

    m_turningEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, frameRateMs);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // Keep an eye on this math, I think we did it wrong, there might not need to be
    // a /60 or /kDriveResolution
    m_driveEncoder.setPositionConversionFactor(2 * Math.PI * kWheelRadius / kDriveEncoderResolution);
    m_driveEncoder.setVelocityConversionFactor(2 * Math.PI * kWheelRadius / 60 / kDriveEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Returns the current velocity of the module.
   *
   * @return The current velocity of the module.
   */
  public double getVelocity() {
    return m_driveEncoder.getVelocity();
  }

  public double getDriveEncoder() {
    return m_driveEncoder.getPosition();
  }

  public double getTurningEncoder() {
    return m_turningEncoder.getPosition();
  }
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getPosition()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);
    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getPosition(),
        state.angle.getRadians());
    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    // Send voltage to the motors
    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }

  public void resetEncoder() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
  }
}
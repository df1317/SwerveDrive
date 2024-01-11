package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADIS16448_IMU;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 14.0; // in meters per second
  public static final double kMaxAngularSpeed = Math.PI * 2; // in radians per second

  // These are the positions of the points where the wheels touch the floor
  // relative to the center of the robot in meters
  private final Translation2d m_frontLeftLocation = new Translation2d(-0.2921, 0.2921);
  private final Translation2d m_frontRightLocation = new Translation2d(0.2921, 0.2921);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.2921, -0.2921);
  private final Translation2d m_backRightLocation = new Translation2d(0.2921, -0.2921);

  private final SwerveModule m_frontLeft = new SwerveModule(11, 31, 51);
  private final SwerveModule m_frontRight = new SwerveModule(12, 32, 52);
  private final SwerveModule m_backLeft = new SwerveModule(21, 41, 61);
  private final SwerveModule m_backRight = new SwerveModule(22, 42, 62);
  // 6.12:1 gear ratio
  private final ADIS16448_IMU m_gyro = new ADIS16448_IMU();

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
      m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      m_kinematics,
      new Rotation2d(m_gyro.getGyroAngleZ()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      }
      );

  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(m_gyro.getGyroAngleZ()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    // cap speed
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        new Rotation2d(m_gyro.getGyroAngleZ()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
  }

  public void dashboard(double[] driveVals) {
    // gyroz
    SmartDashboard.putNumber("gyroz", m_gyro.getGyroAngleZ());

    // front left
    SmartDashboard.putNumber("m_frontLeft_driveEncoder", m_frontLeft.getDriveEncoder());
    SmartDashboard.putNumber("m_frontLeft_turningEncoder", m_frontLeft.getTurningEncoder());

    // front right
    SmartDashboard.putNumber("m_frontRight_driveEncoder", m_frontRight.getDriveEncoder());
    SmartDashboard.putNumber("m_frontRight_turningEncoder", m_frontRight.getTurningEncoder());

    // back left
    SmartDashboard.putNumber("m_backLeft_driveEncoder", m_backLeft.getDriveEncoder());
    SmartDashboard.putNumber("m_backLeft_turningEncoder", m_backLeft.getTurningEncoder());

    // back right
    SmartDashboard.putNumber("m_backRight_driveEncoder", m_backRight.getDriveEncoder());
    SmartDashboard.putNumber("m_backRight_turningEncoder", m_backRight.getTurningEncoder());

    // joysticks
    for (int i = 0; i < driveVals.length; i++) {
      String key = "driveVals" + i;
      SmartDashboard.putNumber(key, driveVals[i]);
    }
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoder();
    m_frontRight.resetEncoder();
    m_backLeft.resetEncoder();
    m_backRight.resetEncoder();
    m_gyro.reset();
  }
}

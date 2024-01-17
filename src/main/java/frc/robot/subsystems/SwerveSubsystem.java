// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16448_IMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
  private final ADIS16448_IMU gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    //instantiates new gyro, wipes it, and zeros it
    gyro = new ADIS16448_IMU();
    zeroGyro();

    //Creates all four swerve modules into a swerve drive
    mSwerveMods =
    new SwerveModule[] {
      new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
      new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
      new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
      new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
    };

    //creates new swerve odometry (odometry is where the robot is on the field)
    swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions());

    //puts out the field
    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    // Display whether the control is field relative on SmartDashboard
    SmartDashboard.putBoolean("Field Relative", fieldRelative);

    // Calculate swerve module states based on control mode (field-relative or robot-centric)
    SwerveModuleState[] swerveModuleStates;
    if (fieldRelative) {
        SmartDashboard.putNumber("Gyro Yaw", getYaw().getDegrees());
        // Use field-relative control if fieldRelative is true
        swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
        );
    } else {
        // Use robot-centric control if fieldRelative is false
        swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
        );
    }

    // Set to top speed if above top speed
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    // Set states for all 4 modules
    for (SwerveModule mod : mSwerveMods) {
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
}

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }


  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public void setWheelsToX() {
    setModuleStates(new SwerveModuleState[] {
      // front left
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
      // front right
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
      // back left
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
      // back right
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135.0))
    });
  }


  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
}


  public void zeroGyro() {
    gyro.reset();
  }

  public Rotation2d getYaw() {
    //fancy if else loop again
    return (Constants.SwerveConstants.invertGyro)
        ? Rotation2d.fromDegrees(360 - (360*gyro.getGyroAngleY()))
        : Rotation2d.fromDegrees(360*gyro.getGyroAngleY());
  }

  public boolean AutoBalance(){
    double roll_error = gyro.getGyroAngleX();//the angle of the robot
    double balance_kp = -.005;//Variable muliplied by roll_error
    double position_adjust = 0.0;
    double min_command = 0.0;//adds a minimum input to the motors to overcome friction if the position adjust isn't enough
    if (roll_error > 6.0)
    {
      position_adjust = balance_kp * roll_error + min_command;//equation that figures out how fast it should go to adjust
      //position_adjust = Math.max(Math.min(position_adjust,.15), -.15);  this gets the same thing done in one line
      if (position_adjust > .1){position_adjust = .1;}
      if (position_adjust < -.1){position_adjust = -.1;}
      drive(new Translation2d(position_adjust, 0), 0.0, true, false);
      
      return false;
    }
    else if (roll_error < -6.0)
    {
      position_adjust = balance_kp * roll_error - min_command;
      drive(new Translation2d(position_adjust, 0), 0.0, true, false);
      if (position_adjust > .3){position_adjust = .3;}
      if (position_adjust < -.3){position_adjust = -.3;}
      return false;
    }
    else{
      drive(new Translation2d(0, 0), 0.0, true, false);
      return true;}
    
  }



  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());

    SmartDashboard.putNumber("Pigeon Roll",  gyro.getGyroRateX());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
  }
}

}

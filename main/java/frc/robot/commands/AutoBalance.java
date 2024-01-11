// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalance extends CommandBase {
  private SwerveSubsystem m_swerveSubsystem;
  public boolean isBalanced;

  /** Creates a new AutoDriveBangBang. */
  public AutoBalance(SwerveSubsystem swerveSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveSubsystem = swerveSub;
    addRequirements(swerveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerveSubsystem.AutoBalance();
    SmartDashboard.putBoolean("Is Balanced?", isBalanced);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.drive(new Translation2d(0,0), 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (isBalanced);
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FiringSubsystem;
import frc.robot.Constants;

public class FireNote extends CommandBase {
    private double startTime;
    private double duration = Constants.SwerveConstants.Firing.Duration;

    private FiringSubsystem m_FiringSubsystem;

    public FireNote(FiringSubsystem FiringSub) {
        m_FiringSubsystem = FiringSub;
        this.duration = Constants.SwerveConstants.Firing.Duration;
        addRequirements(FiringSub);
    }
    
    @Override
    public void initialize() {
        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        // Spin up motors to the specified speed
        m_FiringSubsystem.spinUp();
    }
    
    @Override
    public boolean isFinished() {
        // Check if the specified duration has passed
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime >= duration;
    }

    @Override
    public void end(boolean interrupted) {
        // Spin down motors
        m_FiringSubsystem.spinDown();
    }
}

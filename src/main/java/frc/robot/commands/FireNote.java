package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.FiringSubsystem;
import frc.robot.Constants;

public class FireNote extends CommandBase {
    private double startTime;
    private double duration = Constants.SwerveConstants.Firing.Duration;
    private double speed;
    private double direction = 0;
    private Trigger button;
    private Trigger leftTrigger;
    private Trigger rightTrigger;

    private FiringSubsystem m_FiringSubsystem;

    public FireNote(FiringSubsystem FiringSub, boolean far, Trigger button, Trigger leftTrigger, Trigger rightTrigger) {
        m_FiringSubsystem = FiringSub;
        this.duration = Constants.SwerveConstants.Firing.Duration;
        this.button = button;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
        addRequirements(FiringSub);
        if (far) {
            speed = Constants.SwerveConstants.Firing.FarSpeed;
        }
        else {
            speed = Constants.SwerveConstants.Firing.NearSpeed;
        }
    }
    
    @Override
    public void initialize() {
        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        // Spin up motors to the specified speed
        if (leftTrigger.getAsBoolean() && !rightTrigger.getAsBoolean()) {
            direction = -1;
        } else if (rightTrigger.getAsBoolean() && !leftTrigger.getAsBoolean()) {
            direction = 1;
        } else {
            direction = 0;
        }

        m_FiringSubsystem.spinUp(speed, direction);
    }
    
    @Override
    public boolean isFinished() {
        // Check if the button is released or if the specified duration has passed
        return !button.getAsBoolean() ||
            (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime >= duration);
    }

    @Override
    public void end(boolean interrupted) {
        // Spin down motors
        m_FiringSubsystem.spinDown();
    }
}

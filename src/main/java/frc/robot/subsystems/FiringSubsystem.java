package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class FiringSubsystem extends SubsystemBase {
    private TalonSRX motor0 = new TalonSRX(Constants.SwerveConstants.Firing.MotorID0);
    private TalonSRX motor1 = new TalonSRX(Constants.SwerveConstants.Firing.MotorID1);  

    public void spinUp() {
        motor0.set(TalonSRXControlMode.PercentOutput, Constants.SwerveConstants.Firing.Speed);
        motor1.set(TalonSRXControlMode.PercentOutput, Constants.SwerveConstants.Firing.Speed);
    }

    public void spinDown() {
        motor0.set(TalonSRXControlMode.PercentOutput, 0);
        motor1.set(TalonSRXControlMode.PercentOutput, 0);
    }
}

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class FiringSubsystem extends SubsystemBase {
    private TalonSRX motor0 = new TalonSRX(Constants.SwerveConstants.Firing.MotorID0);
    private TalonSRX motor1 = new TalonSRX(Constants.SwerveConstants.Firing.MotorID1);  
    private double speed0;
    private double speed1;

    public void spinUp(double speed, double direction) {
        if (direction < 0) {
            SmartDashboard.putString("direction", "firing left");
            speed0 = -speed;
            speed1 = speed - Constants.SwerveConstants.Firing.TurnAmount;
        } else if (direction == 0) {
            SmartDashboard.putString("direction", "firing straight");
            speed0 = -speed;
            speed1 = speed;
        } else {
            SmartDashboard.putString("direction", "firing right");
            speed0 = -speed + Constants.SwerveConstants.Firing.TurnAmount;
            speed1 = speed;
        }
        SmartDashboard.putNumber("Firing Motor 0 Speed", speed0);
        SmartDashboard.putNumber("Firing Motor 1 Speed", speed1);
        motor0.set(TalonSRXControlMode.PercentOutput, speed0);
        motor1.set(TalonSRXControlMode.PercentOutput, speed1);
    }

    public void spinDown() {
        motor0.set(TalonSRXControlMode.PercentOutput, 0);
        motor1.set(TalonSRXControlMode.PercentOutput, 0);
    }
}

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;

public class Controller {
    public XboxController ctrl = new XboxController(0);
    private double axisDeadzone = 0.05;

    public double[] getControllerDrive(){
        double xAxis = ctrl.getLeftX(); 
        double yAxis = ctrl.getLeftY();
        double zAxis = ctrl.getRightY();

        double[] output = {xAxis, yAxis, zAxis};

        for(int i = 0; i < output.length; i++){
            double val = output[i];
            output[i] = MathUtil.applyDeadband(Math.abs(Math.pow(val,2)), axisDeadzone);
        }

        return output;
    }

}

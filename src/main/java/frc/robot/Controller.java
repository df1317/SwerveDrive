package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;

public class Controller {
    private XboxController ctrl = new XboxController(0);
    private double axisDeadzone = 0.05;

    /**
     * @return an array of filtered/scaled axis values [x drive, y drive, rotation]
     */
    public double[] getControllerDrive(){
        double xAxis = ctrl.getLeftX(); 
        double yAxis = ctrl.getLeftY();
        double zAxis = ctrl.getRightY();

        double[] output = {xAxis, yAxis, zAxis};

        for(int i = 0; i < output.length; i++){
            double mag = Math.abs(output[i]);
            double sign = Math.signum(output[i]);
            output[i] = sign*MathUtil.applyDeadband(Math.pow(mag,2), axisDeadzone);
        }

        return output;
    }

}

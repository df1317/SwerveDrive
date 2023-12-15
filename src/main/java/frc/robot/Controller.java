package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

public class Controller {
    private XboxController ctrl = new XboxController(0);
    private static double axisDeadzone = 0.05;

    /**
     * @return an array of filtered/scaled axis values [x drive, y drive, rotation]
     */
    public double[] getControllerDrive() {
        double xAxis = ctrl.getLeftX();
        double yAxis = -ctrl.getLeftY();
        double zAxis = -ctrl.getRightX();

        double[] output = { xAxis, yAxis, zAxis };

        for (int i = 0; i < output.length; i++) {
            double mag = Math.abs(output[i]);
            double sign = Math.signum(output[i]);
            output[i] = sign * MathUtil.applyDeadband(Math.pow(mag, 2), axisDeadzone);
        }

        return output;
    }
    public boolean getButtons() {
        return ctrl.getXButton();
    }
}

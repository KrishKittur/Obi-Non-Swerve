package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpiutil.math.MathUtil;

public class AnalogEncoder {

    private final AnalogInput analogEncoder; 
    private final double analogOffset;

    public AnalogEncoder(int channel, double offset) {
        analogEncoder = new AnalogInput(channel);
        this.analogOffset = offset;
    }
    
    public double readEncoder() {
        double analogAngle = (MathUtil.clamp(analogEncoder.getVoltage() / RobotController.getVoltage5V(), 0.0, 1.0) * 2.0 * Math.PI) - analogOffset;
        if (analogAngle < 0.0) {
            analogAngle = 2.0 * Math.PI - Math.abs(analogAngle);
        }
        if (analogAngle > Math.PI) {
            analogAngle = analogAngle - 2.0 * Math.PI;
        }
        return -analogAngle;
    }
     
}

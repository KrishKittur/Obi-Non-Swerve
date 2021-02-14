package frc.robot.drive;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class AnalogEncoder {

    private final AnalogInput analogEncoder; 
    private final double analogOffset;

    public AnalogEncoder(int channel, double offset) {
        analogEncoder = new AnalogInput(channel);
        this.analogOffset = offset;
    }
    
    public double get() {
        double analogAngle = ((analogEncoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI) - analogOffset;
        if (analogAngle < 0.0) {
            analogAngle = 2.0 * Math.PI - Math.abs(analogAngle);
        }
        if (analogAngle > Math.PI) {
            analogAngle = analogAngle - 2.0 * Math.PI;
        }
        return -analogAngle;
    }
     
}

package frc.robot.drive;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpiutil.math.MathUtil;

// Class
public class SwerveModule {

    // Output Enum
    public enum Output {
        PERCENT, VELOCITY
    }

    // Variables
    private final double wheelRadius = 0.0508;
    private final boolean isReversed;
    public final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final CANEncoder driveEncoder;
    private final AnalogEncoder turnEncoder;
    private final PIDController turnPID = new PIDController(0.5, 0.0, 0.0001);
    private final PIDController drivePID = new PIDController(0.5, 0.0, 0.0);
    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.15, 2.9, 0.3);
    private final boolean encoderReversed;

    // Constructor
    public SwerveModule(int driveID, int turnID, int turnChannel, double turnOffset, boolean isReversed, boolean encoderReversed) {
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = new AnalogEncoder(turnChannel, turnOffset);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        this.isReversed = isReversed;
        this.encoderReversed = encoderReversed;
    }

    // Set drive method
    public void setDrivePercent(double dutyCycle) {
        driveMotor.set(isReversed ? -dutyCycle : dutyCycle);
    }

    // Get drive voltage method
    public void setDriveVoltage(double voltage) {
        driveMotor.setVoltage(isReversed ? -voltage : voltage);
    }

    // Get drive velocity method
    public double getDriveVelocity() {
        return encoderReversed ? -(((driveEncoder.getVelocity() * (1.0/7.04))/60.0) * 2 * Math.PI * wheelRadius) : 
        (((driveEncoder.getVelocity() * (1.0/7.04))/60.0) * 2 * Math.PI * wheelRadius);
    }

    // Get state method
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocity(), 
            new Rotation2d(turnEncoder.get())
        );
    }

    // Set desired state method
    public void setDesiredState(SwerveModuleState desiredState, Output output) {
        SwerveModuleState state = SwerveModuleState.optimize(
            desiredState, new Rotation2d(turnEncoder.get())
        );
        double turnOutput = MathUtil.clamp(turnPID.calculate(turnEncoder.get(), state.angle.getRadians()), -0.5, 0.5);
        if (output == Output.PERCENT) {
            setDrivePercent(state.speedMetersPerSecond);
        } else {
            double driveFFOutput = driveFF.calculate(desiredState.speedMetersPerSecond);
            double drivePIDOutput = drivePID.calculate(getDriveVelocity(), desiredState.speedMetersPerSecond);
            double driveOutput = MathUtil.clamp(driveFFOutput + drivePIDOutput, -12, 12);
            setDriveVoltage(driveOutput);
        }
        turnMotor.set(turnOutput);
        if (desiredState.speedMetersPerSecond == 0.0) {
            driveMotor.setIdleMode(IdleMode.kBrake);
        } else {
            driveMotor.setIdleMode(IdleMode.kCoast);
        }
    }  
    
}

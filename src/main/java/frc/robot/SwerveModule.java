package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.controller.PIDController;   
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

public class SwerveModule {

    // Variables
    private final double wheelRadius = 0.0508;
    private final boolean isReversed;

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final CANEncoder driveEncoder;
    private final AnalogEncoder turnEncoder;
    private final PIDController turnPID = new PIDController(0.5, 0.0, 0.0001);

    private final String moduleName;

    private final boolean encoderReversed;

    // Constructor
    public SwerveModule(int driveID, int turnID, int turnChannel, double turnOffset, boolean isReversed, String moduleName, boolean encoderReversed) {
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = new AnalogEncoder(turnChannel, turnOffset);

        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        this.isReversed = isReversed;
        this.moduleName = moduleName;
        this.encoderReversed = encoderReversed;
    }

    // Set Drive method
    public void setDrive(double dutyCycle) {
        driveMotor.set(isReversed ? -dutyCycle : dutyCycle);
    }

    // Get state method
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            encoderReversed ? -(((driveEncoder.getVelocity() * (1.0/7.04))/60.0) * 2 * Math.PI * wheelRadius) : 
            (((driveEncoder.getVelocity() * (1.0/7.04))/60.0) * 2 * Math.PI * wheelRadius), 
            new Rotation2d(turnEncoder.readEncoder())
        );
    }

    // Set desired state method
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(
            desiredState, new Rotation2d(turnEncoder.readEncoder())
        );

        double turnOutput = MathUtil.clamp(
            turnPID.calculate(
                turnEncoder.readEncoder(), 
                state.angle.getRadians()
            ),
            -0.5,
            0.5
        );

        SmartDashboard.putNumber(moduleName + " " + "Desired Angle", state.angle.getRadians());
        SmartDashboard.putNumber(moduleName + " " + "Angle", turnEncoder.readEncoder());

        setDrive(state.speedMetersPerSecond);
        turnMotor.set(turnOutput);
    }  
    
}

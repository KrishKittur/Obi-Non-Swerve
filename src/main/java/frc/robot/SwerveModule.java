package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpiutil.math.MathUtil;

public class SwerveModule {

    // Variables
    private final double wheelRadius = 0.0508;

    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final CANEncoder driveEncoder;
    private final AnalogEncoder turnEncoder;

    private final PIDController drivePID = new PIDController(1, 0, 0);
    private final PIDController turnPID = new PIDController(1, 0, 0);

    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.15, 2.9, 0.3);

    // Constructor
    public SwerveModule(int driveID, int turnID, int turnChannel, int turnOffset) {
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = new AnalogEncoder(turnChannel, turnOffset);

        turnPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    // Get state method
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            ((driveEncoder.getVelocity() * (1.0/7.04))/60.0) * 2 * Math.PI * wheelRadius, 
            new Rotation2d(turnEncoder.readEncoder())
        );
    }

    // Set desired state method
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(
            desiredState, new Rotation2d(turnEncoder.readEncoder())
        );

        double drivePIDOutput = drivePID.calculate(
            ((driveEncoder.getVelocity() * (1.0/7.04))/60.0) * 2 * Math.PI * wheelRadius, 
            state.speedMetersPerSecond
        );

        double driveFFOutput = driveFF.calculate(
            state.speedMetersPerSecond
        );

        double driveOutput = MathUtil.clamp(drivePIDOutput + driveFFOutput, -12, 12);

        double turnOutput = MathUtil.clamp(
            turnPID.calculate(
                turnEncoder.readEncoder(), 
                state.angle.getRadians()
            ),
            -12,
            12
        );

        driveMotor.setVoltage(driveOutput);
        turnMotor.setVoltage(turnOutput);
    }  
    
}

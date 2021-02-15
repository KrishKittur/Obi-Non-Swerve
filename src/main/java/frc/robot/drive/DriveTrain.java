package frc.robot.drive;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.drive.SwerveModule.Output;

// Class
public class DriveTrain {

    // Variables
    private static final double baseWidth = Units.inchesToMeters(20.75);
    private static final double baseLength = Units.inchesToMeters(25.75);
    public static final double centerTurnRadius = Math.sqrt((baseWidth * baseWidth) + (baseLength * baseLength)) / 2.0;
    private final Translation2d frontLeftLocation = new Translation2d(baseLength / 2.0, baseWidth / 2.0);
    private final Translation2d frontRightLocation = new Translation2d(baseLength / 2.0, -baseWidth / 2.0);
    private final Translation2d backLeftLocation = new Translation2d(-baseLength / 2.0, baseWidth / 2.0);
    private final Translation2d backRightLocation = new Translation2d(-baseLength / 2.0, -baseWidth / 2.0);
    public final SwerveModule frontLeft = new SwerveModule(7, 8, 0, Units.degreesToRadians(287.790582), false, false);
    public final SwerveModule frontRight = new SwerveModule(1, 2, 1, Units.degreesToRadians(103.122711), true, false);
    public final SwerveModule backLeft = new SwerveModule(5, 6, 2, Units.degreesToRadians(165.787514), false, false);
    public final SwerveModule backRight = new SwerveModule(3, 4, 3, Units.degreesToRadians(57.719948), true, true);
    public final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
    public final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());
    public Pose2d pose = odometry.getPoseMeters();
    Field2d field = new Field2d();

    // Method to drive
    public void drive(double xSpeed, double ySpeed, double rot, Output output, double maxVel) {
        SwerveModuleState[] states;
        if (output == Output.PERCENT) {
            states = kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
            );
        } else {
            states = kinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, rot)
            );
        }
        if (Math.abs(xSpeed) < 0.01 && Math.abs(ySpeed) < 0.01 && Math.abs(rot) < 0.01) {
            double crossAngle = new Rotation2d(baseLength, baseWidth).getRadians();
            SwerveModuleState flState = new SwerveModuleState(0.0, new Rotation2d(crossAngle));
            SwerveModuleState frState = new SwerveModuleState(0.0, new Rotation2d(-crossAngle));
            SwerveModuleState blState = new SwerveModuleState(0.0, new Rotation2d(-crossAngle));
            SwerveModuleState brState = new SwerveModuleState(0.0, new Rotation2d(crossAngle));
            states[0] = flState;
            states[1] = frState;
            states[2] = blState;
            states[3] = brState;
        }
        SwerveDriveKinematics.normalizeWheelSpeeds(states, maxVel);
        frontLeft.setDesiredState(states[0], output);
        frontRight.setDesiredState(states[1], output);
        backLeft.setDesiredState(states[2], output);
        backRight.setDesiredState(states[3], output);
    }

    // Method to update odometry
    public void updateOdometry() {
        pose = odometry.update(
            gyro.getRotation2d(), 
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        );
    }

    // Method to reset the drive
    public void resetDrive(Pose2d pose) {
        gyro.reset();
        gyro.calibrate();
        odometry.resetPosition(pose, gyro.getRotation2d());
        pose = odometry.getPoseMeters();
    }

    // Method to update the field
    public void updateField() {
        field.setRobotPose(pose);
        SmartDashboard.putData(field);
        System.out.println(pose);
    }

    // Method to set coast
    public void setCoast() {
        frontLeft.driveMotor.setIdleMode(IdleMode.kCoast);
        frontRight.driveMotor.setIdleMode(IdleMode.kCoast);
        backLeft.driveMotor.setIdleMode(IdleMode.kCoast);
        backRight.driveMotor.setIdleMode(IdleMode.kCoast);
    }

    // Method to set brake
    public void setBrake() {
        frontLeft.driveMotor.setIdleMode(IdleMode.kBrake);
        frontRight.driveMotor.setIdleMode(IdleMode.kBrake);
        backLeft.driveMotor.setIdleMode(IdleMode.kBrake);
        backRight.driveMotor.setIdleMode(IdleMode.kBrake);
    }


}

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;

public class DriveTrain {

    // Variables
    private static final double baseWidth = Units.inchesToMeters(20.75);
    private static final double baseLength = Units.inchesToMeters(25.75);
    private static final double centerTurnRadius = Math.sqrt((baseWidth * baseWidth) + (baseLength * baseLength))/2.0;
    public static final double maxSpeed = 12.0/2.9;
    public static final double maxAngularspeed = (12.0/2.9)/centerTurnRadius;

    private final Translation2d frontLeftLocation = new Translation2d(baseLength/2.0, baseWidth/2.0);
    private final Translation2d frontRightLocation = new Translation2d(baseLength/2.0, -baseWidth/2.0);
    private final Translation2d backLeftLocation = new Translation2d(-baseLength/2.0, baseWidth/2.0);
    private final Translation2d backRightLocation = new Translation2d(-baseLength/2.0, -baseWidth/2.0);

    private final SwerveModule frontLeft = new SwerveModule(7, 8, 0, Units.degreesToRadians(287.7), false);
    private final SwerveModule frontRight = new SwerveModule(1, 2, 1, Units.degreesToRadians(102.0), true);
    private final SwerveModule backLeft = new SwerveModule(5, 6, 2, Units.degreesToRadians(165.1), false);
    private final SwerveModule backRight = new SwerveModule(3, 4, 3, Units.degreesToRadians(57.0), true);

    AHRS robotGyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
    );

    // Method to get the angle of the gyro
    public double getAngle() {
        return -robotGyro.getAngle();
    }

    // Method to drive
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        SwerveModuleState[] swerveModuleStates; // For less readability but less lines of code use inline if statement (x ? y : z)
        if (fieldRelative) {
            swerveModuleStates = kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, 
                    ySpeed, 
                    rot, 
                    new Rotation2d(getAngle())
                )
            );
        } else {
            swerveModuleStates = kinematics.toSwerveModuleStates(
                new ChassisSpeeds(
                    xSpeed,
                    ySpeed,
                    rot
                )
            );
        }

        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, maxSpeed);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }
    
}

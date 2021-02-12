package frc.robot;

import java.io.FileWriter;
import java.io.IOException;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.util.Units;

public class DriveTrain {

    // Variables
    private static final double baseWidth = Units.inchesToMeters(20.75);
    private static final double baseLength = Units.inchesToMeters(25.75);
    public static final double centerTurnRadius = Math.sqrt((baseWidth * baseWidth) + (baseLength * baseLength)) / 2.0;

    private final Translation2d frontLeftLocation = new Translation2d(baseLength / 2.0, baseWidth / 2.0);
    private final Translation2d frontRightLocation = new Translation2d(baseLength / 2.0, -baseWidth / 2.0);
    private final Translation2d backLeftLocation = new Translation2d(-baseLength / 2.0, baseWidth / 2.0);
    private final Translation2d backRightLocation = new Translation2d(-baseLength / 2.0, -baseWidth / 2.0);

    private final SwerveModule frontLeft = new SwerveModule(7, 8, 0, Units.degreesToRadians(287.790582), false,
            "Module fl", false);
    private final SwerveModule frontRight = new SwerveModule(1, 2, 1, Units.degreesToRadians(103.122711), true,
            "Module fr", false);
    private final SwerveModule backLeft = new SwerveModule(5, 6, 2, Units.degreesToRadians(165.787514), false,
            "Module bl", false);
    private final SwerveModule backRight = new SwerveModule(3, 4, 3, Units.degreesToRadians(57.719948), true,
            "Module br", true);

    public final AHRS robotGyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation,
            backLeftLocation, backRightLocation);

    public final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, robotGyro.getRotation2d());

    private final Field2d field = new Field2d();

    public final HolonomicDriveController controller = new HolonomicDriveController(new PIDController(1, 0, 0),
            new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(4, 1)));

    private Pose2d pose = odometry.getPoseMeters();

    // Method to drive
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        SwerveModuleState[] swerveModuleStates; // For less readability but less lines of code use inline if statement
                                                // (x ? y : z)
        if (fieldRelative) {
            swerveModuleStates = kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, robotGyro.getRotation2d()));
        } else {
            swerveModuleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        }

        if (Math.abs(xSpeed) < 0.01 && Math.abs(ySpeed) < 0.01 && Math.abs(rot) < 0.01) {
            double crossAngle = new Rotation2d(baseLength, baseWidth).getRadians();
            SwerveModuleState flState = new SwerveModuleState(0.0, new Rotation2d(crossAngle));
            SwerveModuleState frState = new SwerveModuleState(0.0, new Rotation2d(-crossAngle));
            SwerveModuleState blState = new SwerveModuleState(0.0, new Rotation2d(-crossAngle));
            SwerveModuleState brState = new SwerveModuleState(0.0, new Rotation2d(crossAngle));
            swerveModuleStates[0] = flState;
            swerveModuleStates[1] = frState;
            swerveModuleStates[2] = blState;
            swerveModuleStates[3] = brState;
        }

        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, 1.0);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);

        pose = odometry.update(robotGyro.getRotation2d(), frontLeft.getState(), frontRight.getState(),
                backLeft.getState(), backRight.getState());
        field.setRobotPose(pose);
        SmartDashboard.putData(field);
    }

    // Method to follow a trajectory
    public boolean followTrajectory(State desiredState) {
        ChassisSpeeds autoVels = controller.calculate(pose, desiredState, new Rotation2d(0));
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(autoVels);
        SwerveDriveKinematics.normalizeWheelSpeeds(states, 1.0);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);

        pose = odometry.update(robotGyro.getRotation2d(), frontLeft.getState(), frontRight.getState(),
                backLeft.getState(), backRight.getState());

        field.setRobotPose(pose);
        SmartDashboard.putData(field);

        try {
            FileWriter writer = new FileWriter(Filesystem.getOperatingDirectory() + "/data.csv", true);
            writer.write(pose.getX() + ", " + pose.getY() + ", " + desiredState.poseMeters.getX() + ", " + desiredState.poseMeters.getY() + "\n");
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

        return controller.atReference();
    }

}

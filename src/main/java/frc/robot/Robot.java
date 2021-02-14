package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.drive.DriveTrain;
import frc.robot.drive.SwerveModule.Output;
import frc.robot.trajectories.Trajectories;

// Class
public class Robot extends TimedRobot {

  // Variables
  XboxController controller = new XboxController(0);
  DriveTrain swerve = new DriveTrain();
  Timer timer;
  HolonomicDriveController autoController = new HolonomicDriveController(
    new PIDController(0.5, 0, 0), 
    new PIDController(0.5, 0, 0), 
    new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(Units.degreesToRadians(180), Units.degreesToRadians(120)))
  );

  // Robot Init
  @Override
  public void robotInit() {
    swerve.resetDrive(new Pose2d(new Translation2d(), new Rotation2d()));
  }

  // Robot Periodic
  @Override
  public void robotPeriodic() {
    swerve.updateOdometry();
    swerve.updateField();
  }

  // Teleop Periodic
  @Override
  public void teleopPeriodic() {
    double xSpeed = -controller.getY(Hand.kRight);
    double ySpeed = -controller.getX(Hand.kRight);
    double rot = -controller.getX(Hand.kLeft) / DriveTrain.centerTurnRadius;
    swerve.drive(xSpeed, ySpeed, rot, Output.PERCENT, 1.0);
    if (controller.getRawButton(Button.kStart.value)) {
      swerve.resetDrive(new Pose2d(new Translation2d(), new Rotation2d()));
    }
    /* FOR DEBUGGING */
    SmartDashboard.putNumber("fl vel", swerve.frontLeft.getDriveVelocity());
    SmartDashboard.putNumber("fr vel", swerve.frontRight.getDriveVelocity());
    SmartDashboard.putNumber("bl vel", swerve.backLeft.getDriveVelocity());
    SmartDashboard.putNumber("br vel", swerve.backRight.getDriveVelocity());
  }

  // Autonomus Init
  @Override
  public void autonomousInit() {
    timer = new Timer();
    timer.start();
    swerve.resetDrive(Trajectories.simpleTrajectoryTest.getInitialPose());
  }

  // Autonomus Periodic
  @Override
  public void autonomousPeriodic() {
    if (timer.get() < Trajectories.simpleTrajectoryTest.getTotalTimeSeconds()) {
      State desiredPose = Trajectories.simpleTrajectoryTest.sample(timer.get());
      ChassisSpeeds speeds = autoController.calculate(swerve.pose, desiredPose, Rotation2d.fromDegrees(0.0));
      swerve.drive(
        speeds.vxMetersPerSecond, 
        speeds.vyMetersPerSecond, 
        speeds.omegaRadiansPerSecond, 
        Output.VELOCITY, 
        Trajectories.maxVelocity
      );
    } else {
      swerve.drive(0.0, 0.0, 0.0, Output.PERCENT, 0.0);
    }
  }
}


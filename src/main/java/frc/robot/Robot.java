package frc.robot;


import java.util.List;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.util.Units;

public class Robot extends TimedRobot {

  XboxController controller = new XboxController(0);
  DriveTrain swerve = new DriveTrain();
  Timer timer;
  Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(new Translation2d(0, 0), new Rotation2d(Units.degreesToRadians(0))),
    List.of(new Translation2d(1, 1)),
    new Pose2d(new Translation2d(2, 2), new Rotation2d(Units.degreesToRadians(0))),
    new TrajectoryConfig(1.0, 1.0)
  );


  // Teleop Init
  @Override
  public void teleopInit() {
    SmartDashboard.putNumber("Angle", 0.0);
    swerve.robotGyro.reset();
    swerve.robotGyro.calibrate();
    swerve.odometry.resetPosition(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), swerve.robotGyro.getRotation2d());
  }

  // Teleop Periodic
  @Override
  public void teleopPeriodic() {
    double xSpeed = -controller.getY(Hand.kRight);
    double ySpeed = -controller.getX(Hand.kRight);
    double rot = -controller.getX(Hand.kLeft) / DriveTrain.centerTurnRadius;
    swerve.drive(xSpeed, ySpeed, rot, true);
    if (controller.getAButton()) {
      swerve.robotGyro.reset();
      swerve.robotGyro.calibrate();
      swerve.odometry.resetPosition(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), swerve.robotGyro.getRotation2d());
    }
  }

  // Autonomus Init
  @Override
  public void autonomousInit() {
    timer = new Timer();
    timer.start();
    swerve.robotGyro.reset();
    swerve.odometry.resetPosition(trajectory.getInitialPose(), swerve.robotGyro.getRotation2d());
  }

  // Autonomus Periodic
  @Override
  public void autonomousPeriodic() {
    if (timer.get() < trajectory.getTotalTimeSeconds() && !swerve.controller.atReference()) {
      State desiredState = trajectory.sample(timer.get());
      swerve.followTrajectory(desiredState);
    } else {
      swerve.drive(0, 0, 0, true);
    }
  }
}


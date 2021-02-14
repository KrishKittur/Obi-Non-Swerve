package frc.robot.trajectories;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;

// Class
public class Trajectories {

    // Variables
    public static final double maxVelocity = Units.feetToMeters(8);
    public static final double maxAcceleration = Units.feetToMeters(7);

    // Simple Trajectory Test
    public static final Trajectory simpleTrajectoryTest = generateTrajectory(
        maxVelocity, 
        maxAcceleration, 
        List.of(
            new Pose2d(new Translation2d(0, 0), new Rotation2d()),
            new Pose2d(new Translation2d(1, 1), new Rotation2d()),
            new Pose2d(new Translation2d(2, 2), new Rotation2d())
        ),
        true
    );

    // Method to generate a trajectory
    private static Trajectory generateTrajectory(double maxVel, double maxAccel, List<Pose2d> waypoints, boolean clampedCubic) {
        if (waypoints.size() < 1) {
            return new Trajectory();
        }
        TrajectoryConfig config = new TrajectoryConfig(maxVel, maxAccel);
        if (!clampedCubic) {
            return TrajectoryGenerator.generateTrajectory(waypoints, config);
        }
        List<Translation2d> interiorWaypoints = new ArrayList<>();
        for (int i=1; i<waypoints.size() - 1; i++) {
            interiorWaypoints.add(waypoints.get(i).getTranslation());
        }
        return TrajectoryGenerator.generateTrajectory(waypoints.get(0), interiorWaypoints, waypoints.get(waypoints.size() - 1), config);
    }
    
}

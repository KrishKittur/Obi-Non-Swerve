package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

public class Robot extends TimedRobot {

  private static final double baseWidth = 1;
  private static final double baseLength = 1;

  private final Translation2d frontLeftLocation = new Translation2d(baseLength/2.0, baseWidth/2.0);
  private final Translation2d frontRightLocation = new Translation2d(baseLength/2.0, -baseWidth/2.0);
  private final Translation2d backLeftLocation = new Translation2d(-baseLength/2.0, baseWidth/2.0);
  private final Translation2d backRightLocation = new Translation2d(-baseLength/2.0, -baseWidth/2.0);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
  );


  @Override
  public void robotInit() {
    SmartDashboard.putNumber("Vx", 0.0);
    SmartDashboard.putNumber("Vy", 0.0);
    SmartDashboard.putNumber("Omega", 0.0);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    ChassisSpeeds speed = new ChassisSpeeds(SmartDashboard.getNumber("Vx", 0), SmartDashboard.getNumber("Vy", 0), SmartDashboard.getNumber("Omega", 0));
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);
    for (int i=0; i<states.length; i++) {
      SwerveModuleState newState = SwerveModuleState.optimize(states[i], new Rotation2d(Units.degreesToRadians(90)));
      SmartDashboard.putNumber("Module " + i + " Speed", newState.speedMetersPerSecond);
      SmartDashboard.putNumber("Module " + i + " Angle", newState.angle.getDegrees());
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}

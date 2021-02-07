package frc.robot;


import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  XboxController controller = new XboxController(0);
  DriveTrain swerve = new DriveTrain();

  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(2);

  @Override
  public void teleopInit() {
    SmartDashboard.putNumber("Angle", 0.0);
    swerve.robotGyro.reset();
  }

  @Override
  public void teleopPeriodic() {
    double xSpeed = -xSpeedLimiter.calculate(controller.getY(Hand.kLeft)) * DriveTrain.maxSpeed;
    double ySpeed = -ySpeedLimiter.calculate(controller.getX(Hand.kLeft)) * DriveTrain.maxSpeed;
    double rot = -rotLimiter.calculate(controller.getX(Hand.kRight)) * DriveTrain.maxAngularspeed;
    swerve.drive(xSpeed, ySpeed, rot, true);
    System.out.println(swerve.robotGyro.getAngle());

  }
}

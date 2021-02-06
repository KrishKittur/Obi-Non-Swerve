package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Robot extends TimedRobot {

  XboxController controller = new XboxController(0);
  DriveTrain swerve = new DriveTrain();

  @Override
  public void teleopInit() {
    swerve.robotGyro.reset();
  }

  @Override
  public void teleopPeriodic() {
    double xSpeed = -controller.getY(Hand.kLeft) * DriveTrain.maxSpeed;
    double ySpeed = -controller.getX(Hand.kLeft) * DriveTrain.maxSpeed;
    double rot = -controller.getX(Hand.kRight) * DriveTrain.maxAngularspeed;
    swerve.drive(xSpeed, ySpeed, rot, true);
    System.out.println(swerve.robotGyro.getAngle());

  }
}

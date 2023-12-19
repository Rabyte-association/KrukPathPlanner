package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  public  final WPI_TalonSRX l1 = new WPI_TalonSRX(DriveConstants.kLeftMotor1Port);
  private final WPI_TalonSRX l2 = new WPI_TalonSRX(DriveConstants.kLeftMotor2Port);
  private final WPI_TalonSRX r1 = new WPI_TalonSRX(DriveConstants.kRightMotor1Port);
  private final WPI_TalonSRX r2 = new WPI_TalonSRX(DriveConstants.kRightMotor2Port);
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(
          l1,
          l2);

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(
          r1,
          r2);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);


  // The gyro sensor
  private final WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(20);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    l1.setSelectedSensorPosition(0);
    r1.setSelectedSensorPosition(0);
    m_odometry =
        new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), l1.getSelectedSensorPosition()* (0.152 * Math.PI) /(double) 1024/6, -r1.getSelectedSensorPosition() * 0.152 * Math.PI /(double) 1024/6);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),l1.getSelectedSensorPosition()* (0.152 * Math.PI) /(double) 1024/6, -r1.getSelectedSensorPosition()* (0.152 * Math.PI) /(double) 1024/6);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(l1.getSelectedSensorVelocity()*10*(0.152 * Math.PI) /(double) 1024/6,- r1.getSelectedSensorVelocity()*10*(0.152 * Math.PI) /(double) 1024/6);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    l1.setSelectedSensorPosition(0);
    r1.setSelectedSensorPosition(0);
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), -l1.getSelectedSensorVelocity()*10*(0.152 * Math.PI) /(double) 1024/6,- r1.getSelectedSensorVelocity()*10*(0.152 * Math.PI) /(double) 1024/6, pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
    SmartDashboard.putNumber("vel", -l1.getSelectedSensorVelocity()*10*(0.152 * Math.PI) /(double) 1024/6
        );
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
    SmartDashboard.putNumber("vel", -l1.getSelectedSensorVelocity()*10*(0.152 * Math.PI) /(double) 1024/6
        );
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    l1.setSelectedSensorPosition(0);
    r1.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (-l1.getSelectedSensorVelocity()*10*(0.152 * Math.PI) /((double) 1024*6) + r1.getSelectedSensorVelocity()*10*(0.152 * Math.PI) /((double) 1024*6)) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */


  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
 

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
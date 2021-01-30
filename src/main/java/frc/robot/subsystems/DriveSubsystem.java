package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase
{
    private final CANSparkMax _left1 = new CANSparkMax(9, MotorType.kBrushless);
    private final CANSparkMax _left2 = new CANSparkMax(10, MotorType.kBrushless);
    private final CANPIDController _leftController = _left1.getPIDController();
    private final CANEncoder _leftEncoder = _left1.getEncoder();
    private final SpeedControllerGroup _leftGroup = new SpeedControllerGroup(_left1, _left2);

    private final CANSparkMax _right1 = new CANSparkMax(8, MotorType.kBrushless);
    private final CANSparkMax _right2 = new CANSparkMax(7, MotorType.kBrushless);
    private final CANPIDController _rightController = _right1.getPIDController();
    private final CANEncoder _rightEncoder = _right1.getEncoder();
    private final SpeedControllerGroup _rightGroup = new SpeedControllerGroup(_right1, _right2);

    private final DifferentialDrive _drive = new DifferentialDrive(_leftGroup, _rightGroup);

    private final ADIS16448_IMU _imu = new ADIS16448_IMU();

    private final DifferentialDriveOdometry _odometry = new DifferentialDriveOdometry(_imu.getRotation2d());

    @Override
    public void periodic() {
        _odometry.update(_imu.getRotation2d(), _leftEncoder.getPosition(), _rightEncoder.getPosition());
    }

    /**
     * Returns the current estimated pose of the robot
     * 
     * @return The pose
     */
    public Pose2d getPose() {
        return _odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot
     * 
     * @return the current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(_leftEncoder.getVelocity(), _rightEncoder.getVelocity());
    }

    public void resetEncoders() {
        _leftEncoder.setPosition(0);
        _rightEncoder.setPosition(0);
    }

    /**
     * Resets the odometry to the specified pose.
     * 
     * @param pose The pose to set the odometry to.
     */
    public void resetOdometry(Pose2d pose){
        resetEncoders();
        _odometry.resetPosition(pose, _imu.getRotation2d());
    }

    /**
     * Drives the robot using arcade controls.
     * 
     * @param forward the commanded forward movement
     * @param rotation the commanded rotation
     */
    public void arcadeDrive(double forward, double rotation){
        _drive.arcadeDrive(forward, rotation);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     * 
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts){
        _leftGroup.setVoltage(leftVolts);
        _rightGroup.setVoltage(-rightVolts);
        _drive.feed();
    }

    /**
     * Gets the average position of the two encoders.
     * 
     * @return the average of the two encoder positions.
     */
    public double getAverageEncoderDistance() {
        return (_leftEncoder.getPosition() + _rightEncoder.getPosition()) / 2.0;
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        _imu.reset();
    }

    /**
     * Returns the heading of the robot
     * 
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return _imu.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     * 
     * @return the turn rate of the robot in degrees per second.
     */
    public double getTurnRate() {
        return -_imu.getRate();
    }
}
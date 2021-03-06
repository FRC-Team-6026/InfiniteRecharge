package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase
{
    private final CANSparkMax _left1 = new CANSparkMax(9, MotorType.kBrushless);
    private final CANSparkMax _left2 = new CANSparkMax(10, MotorType.kBrushless);
    private final CANEncoder _leftEncoder;
    private final SpeedControllerGroup _leftGroup;

    private final CANSparkMax _right1 = new CANSparkMax(8, MotorType.kBrushless);
    private final CANSparkMax _right2 = new CANSparkMax(7, MotorType.kBrushless);
    private final CANEncoder _rightEncoder;
    private final SpeedControllerGroup _rightGroup;

    private final DifferentialDrive _drive;

    private final ADIS16448_IMU _imu = new ADIS16448_IMU();

    private final DifferentialDriveOdometry _odometry = new DifferentialDriveOdometry(_imu.getRotation2d());

    public Drive() {
        setupSparkMax(_left1, false);
        setupSparkMax(_left2, false);
        setupSparkMax(_right1, false);
        setupSparkMax(_right2, false);
        _leftGroup = new SpeedControllerGroup(_left1, _left2);
        _rightGroup = new SpeedControllerGroup(_right1, _right2);
        _drive = new DifferentialDrive(_leftGroup, _rightGroup);

        _leftEncoder = _left1.getEncoder();
        _rightEncoder = _right1.getEncoder();

        _leftEncoder.setPositionConversionFactor(Constants.kMetersPerMotorRevolution);
        _leftEncoder.setVelocityConversionFactor(Constants.kMetersPerSecondPerRPM);

        _rightEncoder.setPositionConversionFactor(Constants.kMetersPerMotorRevolution);
        _rightEncoder.setVelocityConversionFactor(Constants.kMetersPerSecondPerRPM);

        resetEncoders();
    }

    @Override
    public void periodic() {
        var leftEncoder = _leftEncoder.getPosition();
        var rightEncoder = -_rightEncoder.getPosition();
        var rotation = _imu.getRotation2d();
        _odometry.update(rotation, leftEncoder, rightEncoder);
        SmartDashboard.putNumber("leftEncoder", leftEncoder);
        SmartDashboard.putNumber("rightEncoder", rightEncoder);
        SmartDashboard.putNumber("rotation", rotation.getDegrees());
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
        return new DifferentialDriveWheelSpeeds(_leftEncoder.getVelocity(), -_rightEncoder.getVelocity());
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
        return (_leftEncoder.getPosition() - _rightEncoder.getPosition()) / 2.0;
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

    /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
    public void setMaxOutput(double maxOutput) {
       _drive.setMaxOutput(maxOutput);
    }

    private void setupSparkMax(CANSparkMax controller, boolean inverted)
    {
        controller.restoreFactoryDefaults();
        controller.setIdleMode(IdleMode.kBrake);
        controller.setInverted(inverted);
    }
}
package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.*;

public class Drive extends SubsystemBase
{
    private final CANSparkMax _left1 = new CANSparkMax(9, MotorType.kBrushless);
    private final CANSparkMax _left2 = new CANSparkMax(10, MotorType.kBrushless);
    private final CANEncoder _leftEncoder;
    private final CANPIDController _leftPid;

    private final CANSparkMax _right1 = new CANSparkMax(8, MotorType.kBrushless);
    private final CANSparkMax _right2 = new CANSparkMax(7, MotorType.kBrushless);
    private final CANEncoder _rightEncoder;
    private final CANPIDController _rightPid;

    private DoubleSupplier _speedSupplier;
    private DoubleSupplier _rotationSupplier;

    private final ADIS16448_IMU _imu = new ADIS16448_IMU();

    private final DifferentialDriveOdometry _odometry = new DifferentialDriveOdometry(_imu.getRotation2d());

    public Drive(DoubleSupplier speedSupplier, DoubleSupplier rotationSupplier) {
        super();
        _speedSupplier = speedSupplier;
        _rotationSupplier = rotationSupplier;

        setupSparkMax(_left1, false);
        setupSparkMax(_left2, false);
        setupSparkMax(_right1, true);
        setupSparkMax(_right2, true);
        _left2.follow(_left1);
        _right2.follow(_right1);

        _leftPid = _left1.getPIDController();
        _rightPid = _right1.getPIDController();

        _leftEncoder = _left1.getEncoder();
        _rightEncoder = _right1.getEncoder();

        setupPid(_leftPid, _leftEncoder);
        setupPid(_rightPid, _rightEncoder);

        _leftEncoder.setPositionConversionFactor(Constants.kMetersPerMotorRevolution);
        _leftEncoder.setVelocityConversionFactor(Constants.kMetersPerSecondPerRPM);

        _rightEncoder.setPositionConversionFactor(Constants.kMetersPerMotorRevolution);
        _rightEncoder.setVelocityConversionFactor(Constants.kMetersPerSecondPerRPM);

        resetEncoders();

        _left1.burnFlash();
        _left2.burnFlash();
        _right1.burnFlash();
        _right2.burnFlash();

        this.setDefaultCommand(new FunctionalCommand(() -> {/*do nothing on init*/},
          // do arcade drive by default
          () -> {arcadeDrive();},
          //when interrupted set PID controls to voltage and default to 0 to stop
          interrupted ->
          {
            _leftPid.setReference(0, ControlType.kVoltage);
            _rightPid.setReference(0, ControlType.kVoltage);
          },
          //never end
          () -> {return false;},
          this));
    }

    @Override
    public void periodic() {
        var leftEncoder = _leftEncoder.getPosition();
        var rightEncoder = _rightEncoder.getPosition();
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
    public void arcadeDrive() {
        var speed = _speedSupplier.getAsDouble();
        var rotation = _rotationSupplier.getAsDouble();

        var leftLinearRpm = speed * Constants.kMaxRpm;
        var rightLinearRpm = speed * Constants.kMaxRpm;
        var leftRotationRpm = rotation * Constants.kRotationDiffRpm;
        var rightRotationRpm = -rotation * Constants.kRotationDiffRpm;

        var leftVelocityRpm = leftLinearRpm + leftRotationRpm;
        var rightVelocityRpm = rightLinearRpm + rightRotationRpm;

        var leftFeedForward = leftVelocityRpm * Constants.kvVoltMinutesPerMotorRotation;
        if (leftFeedForward != 0) {
          leftFeedForward = leftFeedForward > 0 ? leftFeedForward + Constants.ksVolts : leftFeedForward - Constants.ksVolts;
        }
        var rightFeedForward = rightVelocityRpm * Constants.kvVoltMinutesPerMotorRotation;
        if (rightFeedForward != 0) {
          rightFeedForward = rightFeedForward > 0 ? rightFeedForward + Constants.ksVolts : rightFeedForward - Constants.ksVolts;
        }

        _leftPid.setReference(leftVelocityRpm, ControlType.kVelocity, 0, leftFeedForward, ArbFFUnits.kVoltage);
        _rightPid.setReference(rightVelocityRpm, ControlType.kVelocity, 0, rightFeedForward, ArbFFUnits.kVoltage);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     * 
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts){
        _leftPid.setReference(leftVolts, ControlType.kVoltage);
        _rightPid.setReference(rightVolts, ControlType.kVoltage);
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

    public void setSpeedSupplier(DoubleSupplier speedSupplier) {
        _speedSupplier = speedSupplier;
    }

    public void setRotationSupplier(DoubleSupplier rotationSupplier) {
        _rotationSupplier = rotationSupplier;
    }

    private void setupSparkMax(CANSparkMax motor, boolean inverted)
    {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(inverted);
    }

    private void setupPid(CANPIDController controller, CANEncoder encoder){
        controller.setFF(Constants.kFf);
        controller.setP(Constants.kP);
        controller.setI(Constants.kI);
        controller.setD(Constants.kD);
        controller.setOutputRange(Constants.minOutput, Constants.maxOutput);
        controller.setSmartMotionMaxVelocity(Constants.kMaxRpm, 0);
        controller.setSmartMotionMaxAccel(Constants.kMaxAccelRpmPerSec, 0);
        controller.setFeedbackDevice(encoder);
    }
}
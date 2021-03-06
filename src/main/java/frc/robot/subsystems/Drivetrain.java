package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {
    private static final double kGearRatio = 10.71;
    private static final double kWheelDiameterMm = 152.4;
    private static final double kMmPerRevolution = (1 / kGearRatio) * Math.PI * kWheelDiameterMm;
    private static final double kDriveBaseMm = 556.25;
    private static final double kDriveRadiusMm = kDriveBaseMm / 2.0;
    private static final double kP = 5e-5;
    private static final double kI = 1e-6;
    private static final double kD = 0;
    private static final double kF = 0;
    private static final double kDeadbandBottom = 0.17;
    private static final double kDeadbandTop = 0.95;
    private static final double kDeadbandRatio = 1 / (kDeadbandTop - kDeadbandBottom);
    private static final double kMaxRpm = 1500;
    private static final double kRotationDiffRpm = 1000;
    private final CANSparkMax _leftFront = new CANSparkMax(9, MotorType.kBrushless);
    private final CANSparkMax _rightFront = new CANSparkMax(8, MotorType.kBrushless);
    private final CANSparkMax _leftRear = new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax _rightRear = new CANSparkMax(7, MotorType.kBrushless);
    private final CANPIDController _leftController = _leftRear.getPIDController();
    private final CANPIDController _rightController = _rightRear.getPIDController();
    private final CANEncoder _leftCanEncoder = _leftRear.getEncoder();
    private final CANEncoder _rightCanEncoder = _rightRear.getEncoder();

    public void init(){
        _leftFront.restoreFactoryDefaults();
        _rightFront.restoreFactoryDefaults();
        _leftRear.restoreFactoryDefaults();
        _rightRear.restoreFactoryDefaults();

        _leftFront.follow(_leftRear);
        _rightFront.follow(_rightRear);

        _rightRear.setInverted(true);

        //_leftCanEncoder.setInverted(true);
        //_rightCanEncoder.setInverted(true);

        config(_leftController, _leftCanEncoder);
        config(_rightController, _rightCanEncoder);

        _leftFront.burnFlash();
        _rightFront.burnFlash();
        _leftRear.burnFlash();
        _rightRear.burnFlash();
    }

    public void arcadeDrive(double speed, boolean filterSpeedDeadband, double rotation, boolean filterRotationDeadband){
        var filterSpeed = filterInput(speed, filterSpeedDeadband);
        var filterRotation = filterInput(rotation, filterRotationDeadband);

        var leftLinearRpm = filterSpeed * kMaxRpm;
        var rightLinearRpm = filterSpeed * kMaxRpm;
        var leftRotationRpm = filterRotation * kRotationDiffRpm;
        var rightRotationRpm = -filterRotation * kRotationDiffRpm;

        var leftVelocityRpm = leftLinearRpm + leftRotationRpm;
        var rightVelocityRpm = rightLinearRpm + rightRotationRpm;

        //limit velocity to +- kMaxRpm
        leftVelocityRpm = Math.min(leftVelocityRpm, kMaxRpm);
        leftVelocityRpm = Math.max(leftVelocityRpm, -kMaxRpm);
        rightVelocityRpm = Math.min(rightVelocityRpm, kMaxRpm);
        rightVelocityRpm = Math.max(rightVelocityRpm, -kMaxRpm);

        SmartDashboard.putNumber("speed", filterSpeed);
        SmartDashboard.putNumber("rotation", filterRotation);

        SmartDashboard.putNumber("Left target RPM", leftVelocityRpm);
        SmartDashboard.putNumber("right target RPM", rightVelocityRpm);

        SmartDashboard.putNumber("left encoder velocity", _leftCanEncoder.getVelocity());
        SmartDashboard.putNumber("right encoder velocity", _rightCanEncoder.getVelocity());

        SmartDashboard.putNumber("right front current", _rightFront.getOutputCurrent());
        SmartDashboard.putNumber("right rear current", _rightRear.getOutputCurrent());
        SmartDashboard.putNumber("left front current", _leftFront.getOutputCurrent());
        SmartDashboard.putNumber("left rear current", _leftRear.getOutputCurrent());

        SmartDashboard.putNumber("right front temp", _rightFront.getMotorTemperature());
        SmartDashboard.putNumber("right rear temp", _rightRear.getMotorTemperature());
        SmartDashboard.putNumber("left front temp", _leftFront.getMotorTemperature());
        SmartDashboard.putNumber("left rear temp", _leftRear.getMotorTemperature());

        _leftController.setReference(leftVelocityRpm, ControlType.kVelocity);
        _rightController.setReference(rightVelocityRpm, ControlType.kVelocity);
    }

    public void driveDistanceMm(double distanceMm){

        var revolutions = distanceMm / kMmPerRevolution;
        _leftCanEncoder.setPosition(0);
        _rightCanEncoder.setPosition(0);
        _leftController.setReference(revolutions, ControlType.kSmartMotion);
        _rightController.setReference(revolutions, ControlType.kSmartMotion);
    }

    public void turnAngle(double angle){
        var angleRadians = angle * Math.PI / 180.0;
        var rotationDistanceMm = angleRadians * kDriveRadiusMm;
        var revolutions = rotationDistanceMm / kMmPerRevolution;
        _leftCanEncoder.setPosition(0);
        _rightCanEncoder.setPosition(0);
        _leftController.setReference(revolutions, ControlType.kSmartMotion);
        _rightController.setReference(-revolutions, ControlType.kSmartMotion);
    }

    private void config(CANPIDController controller, CANEncoder encoder){
        //encoder.setVelocityConversionFactor(1.0/4096.0);
        controller.setFF(kF);
        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
        controller.setOutputRange(-1, 1);
        controller.setSmartMotionMaxVelocity(kMaxRpm, 0);
        controller.setSmartMotionMaxAccel(1500, 0);
        controller.setFeedbackDevice(encoder);
    }

    /**
     * Filters an input and filters for deadband optionally. Squares the input value
     * and if filtering for deadband normalizes the output -1 to 1.
     * @param input input to be filtered
     * @param filterDeadband whether or not to filter deadband
     * @return returns a normalized value -1 to 1 where +- kDeadbandBottom normalizes to 0
     * and +- kDeadbandTop normalizes to +-1
     */
    private double filterInput(double input, boolean filterDeadband){
        if(filterDeadband && Math.abs(input) <= kDeadbandBottom){
            return 0;
        } else {
            var filteredInput = 0.0;
            if (filterDeadband){
                filteredInput = (input - kDeadbandBottom) * kDeadbandRatio;
            } else {
                filteredInput = input;
            }
            if (filteredInput < 0) {
                return Math.max(filteredInput * filteredInput * -1, -1);
            } else {
                return Math.min(filteredInput * filteredInput, 1);
            }
        }
    }
}
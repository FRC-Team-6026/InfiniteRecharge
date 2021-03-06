package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private final WPI_TalonSRX _top = new WPI_TalonSRX(4);
    private final WPI_TalonSRX _bottom = new WPI_TalonSRX(3);
    private final Solenoid _flipper = new Solenoid(15, 1);
    private final int _pidSlot = 0;
    private final int _timeoutMs = 100;
    private final double _peakOutput = 1;
    private final double _maxVelocityPulsesPer100ms = 27000;
    private final double _speedDiff = 2000;
    private final double _p = 0.25;
    private final double _i = 0.001;
    private final double _d = 20;
    private final double _fBottom = 1023/(_maxVelocityPulsesPer100ms);
    private final double _fTop = 1023/(_maxVelocityPulsesPer100ms - _speedDiff);
    private final int _iZone = 300;
    private final double _maxDiff = 1000;

    public void init(){
        _top.configFactoryDefault();
        _bottom.configFactoryDefault();

        _top.setNeutralMode(NeutralMode.Coast);
        _bottom.setNeutralMode(NeutralMode.Coast);

        _top.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, _pidSlot, _timeoutMs);
        _bottom.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, _pidSlot, _timeoutMs);

        _top.setInverted(false);
        _top.setSensorPhase(false);
        _bottom.setInverted(false);
        _bottom.setSensorPhase(false);

        _top.configNominalOutputForward(0, _timeoutMs);
        _top.configNominalOutputReverse(0, _timeoutMs);
        _top.configPeakOutputForward(_peakOutput, _timeoutMs);
        _top.configPeakOutputReverse(-_peakOutput, _timeoutMs);

        _bottom.configNominalOutputForward(0, _timeoutMs);
        _bottom.configNominalOutputReverse(0, _timeoutMs);
        _bottom.configPeakOutputForward(_peakOutput, _timeoutMs);
        _bottom.configPeakOutputReverse(-_peakOutput, _timeoutMs);

        _top.config_kF(_pidSlot, _fTop, _timeoutMs);
        _top.config_kP(_pidSlot, _p, _timeoutMs);
        _top.config_kI(_pidSlot, _i, _timeoutMs);
        _top.config_kD(_pidSlot, _d, _timeoutMs);
        _top.config_IntegralZone(_pidSlot, _iZone, _timeoutMs);

        _bottom.config_kF(_pidSlot, _fBottom, _timeoutMs);
        _bottom.config_kP(_pidSlot, _p, _timeoutMs);
        _bottom.config_kI(_pidSlot, _i, _timeoutMs);
        _bottom.config_kD(_pidSlot, _d, _timeoutMs);
        _bottom.config_IntegralZone(_pidSlot, _iZone, _timeoutMs);

        _top.configClosedloopRamp(0.5, _timeoutMs);
        _bottom.configClosedloopRamp(0.5, _timeoutMs);
    }

    /**
     * Runs the firing mechanism at the set power
     * @param power power output -1 to 1
     */
    public void fire(double power){

        var velocities = getVelocities(power);
        _top.set(ControlMode.Velocity, velocities[1]);
        _bottom.set(ControlMode.Velocity, -velocities[0]);

        SmartDashboard.putNumber("Target velocity pulses/100ms", velocities[0]);
        SmartDashboard.putNumber("Top velocity pulses/100ms", _top.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Bottom velocity pulses/100ms", _bottom.getSelectedSensorVelocity());
    }

    public boolean isAtSetPower(double power){
        var velocities = getVelocities(power);
        var feedbackVelocity = _bottom.getSelectedSensorVelocity();
        var feedbackTopVelocity = _top.getSelectedSensorVelocity();
        var bottomDiff = Math.abs(velocities[0] + feedbackVelocity);
        var topDiff = Math.abs(velocities[1] - feedbackTopVelocity);
        if (bottomDiff < _maxDiff && topDiff < _maxDiff){
            return true;
        } else {
            return false;
        }
    }

    /**
     * Opens the gate
     */
    public void openGate(){
        _flipper.set(true);
    }

    /**
     * Closes the gate
     */
    public void closeGate(){
        _flipper.set(false);
    }

    private double[] getVelocities(double power)
    {
        var velocity = power * _maxVelocityPulsesPer100ms;
        var velocityTop = Math.max(velocity - _speedDiff, 0);
        double[] velocities = {velocity, velocityTop};
        return velocities;
    }
}
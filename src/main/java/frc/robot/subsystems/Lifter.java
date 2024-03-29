package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lifter extends SubsystemBase {
    private static final double _topEncoderPosition = 440000;
    private static final int _pidLoopIndex = 0;
    private static final int _timeoutMs = 100;
    private static final double kP = 5e-2;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kF = 0;
    private static final double _gearRatio = 20.0;
    private static final double _spoolDiameterMm = 22.0;
    private static final double _spoolCircumferenceMm = _spoolDiameterMm * Math.PI;
    private static final double _pulsesPerRevolution = 2048.0;
    private static final double _100msPerSec = 6;
    private static final double _mmPerSecToPulsesPer100ms = (1 / _spoolCircumferenceMm) * _gearRatio * _pulsesPerRevolution / _100msPerSec;
    private final TalonFX _lightSaber = new TalonFX(20);

    public Lifter(){
        super();
        _lightSaber.configFactoryDefault();
        _lightSaber.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        _pidLoopIndex,
        _timeoutMs);

        _lightSaber.configNominalOutputForward(0, _timeoutMs);
        _lightSaber.configNominalOutputReverse(0, _timeoutMs);
        _lightSaber.configPeakOutputForward(1, _timeoutMs);
        _lightSaber.configPeakOutputReverse(-1, _timeoutMs);

        _lightSaber.config_kF(_pidLoopIndex, kF, _timeoutMs);
        _lightSaber.config_kP(_pidLoopIndex, kP, _timeoutMs);
        _lightSaber.config_kI(_pidLoopIndex, kI, _timeoutMs);
        _lightSaber.config_kD(_pidLoopIndex, kD, _timeoutMs);

        _lightSaber.setNeutralMode(NeutralMode.Brake);

        _lightSaber.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        _lightSaber.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled);

        _lightSaber.setInverted(true);

        _lightSaber.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        var lifterPosition = _lightSaber.getSelectedSensorPosition();
        var lifterVelocity = _lightSaber.getSelectedSensorVelocity();
        SmartDashboard.putNumber("lifter position", lifterPosition);
        SmartDashboard.putNumber("lifter velocity", lifterVelocity);
        if (lifterPosition > _topEncoderPosition && lifterVelocity > 0){
            _lightSaber.set(TalonFXControlMode.Velocity, 0);
        }
        if (_lightSaber.getSensorCollection().isRevLimitSwitchClosed() == 0){
            _lightSaber.setSelectedSensorPosition(0);
        }
    }

    public void move(double mmPerSec){
        var pulsesPer100ms = mmPerSec * _mmPerSecToPulsesPer100ms;
        if (_lightSaber.getSelectedSensorPosition() > _topEncoderPosition && mmPerSec > 0){
            _lightSaber.set(TalonFXControlMode.Velocity, 0);
        }else{
            _lightSaber.set(TalonFXControlMode.Velocity, pulsesPer100ms);
        }
    }
}
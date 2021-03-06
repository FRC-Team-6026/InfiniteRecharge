package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;

public class Lifter extends SubsystemBase {
    private static final double kP = 5e-5;
    private static final double kI = 1e-6;
    private static final double kD = 0;
    private static final double kF = 0;
    private static final double kMaxRpm = 1000;
    private final TalonSRX _lightSaber = new TalonSRX(6);
    private final CANSparkMax _lifter = new CANSparkMax(12, MotorType.kBrushless);
    private final CANPIDController _liftController = _lifter.getPIDController();
    private final CANEncoder _lifterCanEncoder = _lifter.getEncoder();

    public void init(){
        _lifter.restoreFactoryDefaults();
        _lightSaber.configFactoryDefault();

        _lightSaber.setNeutralMode(NeutralMode.Brake);
        _lightSaber.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

        _liftController.setFF(kF);
        _liftController.setP(kP);
        _liftController.setI(kI);
        _liftController.setD(kD);
        _liftController.setOutputRange(-1, 1);
        _liftController.setFeedbackDevice(_lifterCanEncoder);

        _lifter.burnFlash();
    }

    public void moveHook(double power){
        _lightSaber.set(ControlMode.PercentOutput, power);
    }
    
    public void moveRobot(double power){
        var velocity = kMaxRpm * power;
        _liftController.setReference(velocity, ControlType.kVelocity);
    }
}
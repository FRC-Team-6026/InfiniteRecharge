package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final WPI_VictorSPX _intake = new WPI_VictorSPX(14);
    private final CANSparkMax _omniRoller = new CANSparkMax(13, MotorType.kBrushless);
    private final CANEncoder _omniEncoder;
    private final CANPIDController _omniPID;
    private final Solenoid _arms = new Solenoid(15,0);
    private boolean _isExtended = false;
    private final double _maxRollerSpeedRpm = 3000;
    private final double _maxRollerAccelRpm = 1500;

    public Intake()
    {
        super();
        _intake.configFactoryDefault();
        _omniRoller.restoreFactoryDefaults();
        _omniRoller.setIdleMode(IdleMode.kCoast);
        _omniEncoder = _omniRoller.getEncoder();
        _omniPID = _omniRoller.getPIDController();
        _omniPID.setFF(0);
        _omniPID.setP(5e-5);
        _omniPID.setI(0);
        _omniPID.setD(0);
        _omniPID.setOutputRange(-1, 1);
        _omniPID.setSmartMotionMaxVelocity(_maxRollerSpeedRpm, 0);
        _omniPID.setSmartMotionMaxAccel(_maxRollerAccelRpm, 0);
        _omniPID.setFeedbackDevice(_omniEncoder);
        _omniRoller.burnFlash();
    }

    public boolean isExtended(){
        return _isExtended;
    }

    /**
     * Extends moves the arms of the intake to not extended or extended
     */
    public void moveArms(boolean extend){
        _isExtended = extend;
        _arms.set(_isExtended);
    }

    /**
     * Run motors of the intake
     */
    public void run(){
        _intake.set(ControlMode.PercentOutput, 0.6);
        _omniPID.setReference(_maxRollerSpeedRpm, ControlType.kVelocity);
        SmartDashboard.putNumber("Omni Roller speed", _omniEncoder.getVelocity());
    }

    /**
     * Stop the intake motor
     */
    public void stop(){
        _intake.stopMotor();
        _omniPID.setReference(0, ControlType.kVelocity);
    }

    /**
     * Run the intake backwards
     */
    public void reverse(){
        _intake.set(ControlMode.PercentOutput, -0.6);
        _omniPID.setReference(-_maxRollerSpeedRpm, ControlType.kVelocity);
    }
}
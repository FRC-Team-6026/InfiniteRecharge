package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;

public class Intake {

    private final WPI_VictorSPX _intake = new WPI_VictorSPX(14);
    private final Solenoid _arms = new Solenoid(15,0);
    private boolean _isExtended = false;

    public void init(){
        _intake.configFactoryDefault();
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
        _intake.set(ControlMode.PercentOutput, 0.4);
    }

    /**
     * Stop the intake motor
     */
    public void stop(){
        _intake.stopMotor();
    }

    /**
     * Run the intake backwards
     */
    public void reverse(){
        _intake.set(ControlMode.PercentOutput, -0.4);
    }
}
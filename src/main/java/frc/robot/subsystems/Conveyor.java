package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.*;

public class Conveyor extends SubsystemBase {
    private final I2C.Port _i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 _colorSensor = new ColorSensorV3(_i2cPort);
    private final VictorSPX _motor = new VictorSPX(2);
    private final BooleanSupplier _shooterIsAtSetPower;
    private final DigitalInput _shooterPhotocell = new DigitalInput(9);

    public Conveyor(BooleanSupplier shooterIsAtSetPower){
        super();
        _shooterIsAtSetPower = shooterIsAtSetPower;
        _motor.configFactoryDefault();

        this.setDefaultCommand(new RunCommand(() ->
        {
            if (_shooterIsAtSetPower.getAsBoolean() || (isBallInLoadingPosition() && !isBallAtShooterPosition())) {
                run(0.5);
            } else {
                run(0);
            }
        }, this));
    }

    public boolean isBallInLoadingPosition(){
        var r = _colorSensor.getRed();
        var g = _colorSensor.getGreen();
        var b = _colorSensor.getBlue();
        var x = _colorSensor.getProximity();
        SmartDashboard.putNumber("red", r);
        SmartDashboard.putNumber("green", g);
        SmartDashboard.putNumber("blue", b);
        SmartDashboard.putNumber("proximity", x);
        if (x > 130 && r > 180 && g > 300){
            return true;
        }
        return false;
    }

    public boolean isBallAtShooterPosition(){
        return !_shooterPhotocell.get();
    }

    public void run(double percentOutput){
        _motor.set(ControlMode.PercentOutput, percentOutput);
    }
}
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightController extends SubsystemBase {
    private final NetworkTableInstance _instance = NetworkTableInstance.getDefault();
    private final NetworkTable _table = _instance.getTable("limelight");
    private final PIDController _targetPid = new PIDController(0.05, 0, 0);

    public void init(){
        _targetPid.setSetpoint(0);
        _targetPid.setTolerance(0.1);
        SendableRegistry.setName(_targetPid, "limelight Controller", "Target PID");
        turnLightOff();
    }

    /**
     * Calculates the rotation necessary to track the upper
     * target
     * @return
     */
    public double trackTarget() {
        var txEntry= _table.getEntry("tx");
        var xError = txEntry.getNumber(0);
        var rotation = -_targetPid.calculate(xError.doubleValue());
        rotation = Math.min(rotation, 1);
        rotation = Math.max(rotation, -1);
        return rotation;
    }

    public void turnLightOn(){
        var ledEntry = _table.getEntry("ledMode");
        ledEntry.setNumber(3);
    }

    public void turnLightOff(){
        var ledEntry = _table.getEntry("ledMode");
        ledEntry.setNumber(1);
    }
}
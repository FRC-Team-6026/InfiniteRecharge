package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class SlalomCommand {

    private final Trajectory _slalomPath = getTrajectory("paths/output/Slalom.wpilib.json");
    
    public Command getCommand(Drive drive) {

        RamseteCommand slalomCommand = new RamseteCommand(
            _slalomPath,
            drive::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            drive::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            drive::tankDriveVolts,
            drive
        );

        var resetOdometry = new InstantCommand(() -> {
            drive.resetOdometry(_slalomPath.getInitialPose());
        });
        var slalomAndThen = slalomCommand.andThen(() -> {
            slalomCommand.andThen(() -> drive.tankDriveVolts(0, 0));
        });
        return new SequentialCommandGroup(resetOdometry,
          slalomAndThen);
    }

    private Trajectory getTrajectory(String trajectoryJson) {
        Trajectory trajectory = null;
        try {
            var trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJson);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJson, ex.getStackTrace());
        }

        return trajectory;
    }
}

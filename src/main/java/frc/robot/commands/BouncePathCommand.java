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

public class BouncePathCommand {
    private final Trajectory _bounce1 = getTrajectory("paths/output/Bounce1.wpilib.json");
    private final Trajectory _bounce2 = getTrajectory("paths/output/Bounce2.wpilib.json");
    private final Trajectory _bounce3 = getTrajectory("paths/output/Bounce3.wpilib.json");
    private final Trajectory _bounce4 = getTrajectory("paths/output/Bounce4.wpilib.json");

    public BouncePathCommand() {
        super();
    }

    public Command getCommand(Drive drive) {
        var bounce1Ramsete = getRamseteCommand(_bounce1, drive).andThen(() -> drive.tankDriveVolts(0, 0));

        var bounce2Ramsete = getRamseteCommand(_bounce2, drive).andThen(() -> drive.tankDriveVolts(0, 0));

        var bounce3Ramsete = getRamseteCommand(_bounce3, drive).andThen(() -> drive.tankDriveVolts(0, 0));

        var bounce4Ramsete = getRamseteCommand(_bounce4, drive).andThen(() -> drive.tankDriveVolts(0, 0));

        var resetBounce1 = new InstantCommand(() -> {
            drive.resetOdometry(_bounce1.getInitialPose());
        });

        var resetBounce2 = new InstantCommand(() -> {
            drive.resetOdometry(_bounce2.getInitialPose());
        });

        var resetBounce3 = new InstantCommand(() -> {
            drive.resetOdometry(_bounce3.getInitialPose());
        });

        var resetBounce4 = new InstantCommand(() -> {
            drive.resetOdometry(_bounce4.getInitialPose());
        });

        return new SequentialCommandGroup(resetBounce1,
        bounce1Ramsete,
        resetBounce2,
        bounce2Ramsete,
        resetBounce3,
        bounce3Ramsete,
        resetBounce4,
        bounce4Ramsete);
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
    

    private RamseteCommand getRamseteCommand(Trajectory trajectory, Drive drive) {
        var ramseteCommand = new RamseteCommand(
            trajectory,
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

        return ramseteCommand;
    }
}

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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class BouncePathCommand {
    private final Trajectory bounce1 = getTrajectory("paths/output/Bounce1.wpilib.json");
    private final Trajectory bounce2 = getTrajectory("paths/output/Bounce2.wpilib.json");
    private final Trajectory bounce3 = getTrajectory("paths/output/Bounce3.wpilib.json");
    private final Trajectory bounce4 = getTrajectory("paths/output/Bounce4.wpilib.json");

    public BouncePathCommand() {
        super();
    }

    public Command getCommand(Drive drive) {
        var bounce1Ramsete = getRamseteCommand(bounce1, drive);

        var bounce2Ramsete = getRamseteCommand(bounce2, drive);

        var bounce3Ramsete = getRamseteCommand(bounce3, drive);

        var bounce4Ramsete = getRamseteCommand(bounce4, drive);

        // Reset odometry to the starting pose of the trajectory.
        drive.resetOdometry(bounce1.getInitialPose());

        // Run path following command, then stop at the end.
        return bounce1Ramsete.andThen(
            () -> {
                drive.tankDriveVolts(0, 0);
                drive.resetOdometry(bounce2.getInitialPose());
                bounce2Ramsete.andThen(() -> {
                    drive.tankDriveVolts(0, 0);
                    drive.resetOdometry(bounce3.getInitialPose());
                    bounce3Ramsete.andThen(() -> {
                        drive.tankDriveVolts(0, 0);
                        drive.resetOdometry(bounce4.getInitialPose());
                        bounce4Ramsete.andThen(() -> {
                            drive.tankDriveVolts(0, 0);
                        }).execute();;
                    }).execute();;
                }).execute();;
            });
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
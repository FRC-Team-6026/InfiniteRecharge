// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.BarrelRaceCommand;
import frc.robot.commands.SampleTrajectoryCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final double kLowerDeadband = 0.17;
  private static final double kUpperDeadband = 0.95;
  private final XboxController _controller = new XboxController(0);
  // The robot's subsystems and commands are defined here...
  private final Drive _drive = new Drive(() -> filterControllerInputs(-_controller.getY(Hand.kLeft)),
    () -> filterControllerInputs(_controller.getX(Hand.kRight)));
  private final Shooter _shooter = new Shooter(() -> _controller.getTriggerAxis(Hand.kRight));
  private final Conveyor _conveyor = new Conveyor(() -> _shooter.isAtSetPower());
  private final Intake _intake = new Intake();

  private final SampleTrajectoryCommand _sampleTrajectory = new SampleTrajectoryCommand();
  private final BarrelRaceCommand _barrelRaceCommand = new BarrelRaceCommand();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    var rightBumper = new JoystickButton(_controller, XboxController.Button.kBumperRight.value);
    var leftBumper = new JoystickButton(_controller, XboxController.Button.kBumperLeft.value);
    var aButton = new JoystickButton(_controller, XboxController.Button.kA.value);

    rightBumper.whenPressed(new InstantCommand(() -> {_conveyor.run(0.3);}, _conveyor), true)
      .whenReleased(new InstantCommand(() -> {_conveyor.run(0);}, _conveyor));

    leftBumper.whenPressed(new InstantCommand(() -> {_intake.run();}, _intake),true)
      .whenReleased(new InstantCommand(() -> {_intake.stop();}, _intake), true);

    aButton.whenPressed(new InstantCommand(() -> {
      _intake.moveArms(!_intake.isExtended());
    }, _intake), true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return _barrelRaceCommand.getCommand(_drive);
  }

  private static double filterControllerInputs(double input) {
    if (Math.abs(input) < kLowerDeadband){
      return 0;
    }

    //The XBox controller sometimes won't go all the way to 1
    if (Math.abs(input) > kUpperDeadband){
      return input > 0 ? 1 : -1;
    }

    //cubing input so we have more control at lower values
    var filtered = input * input * input;

    return filtered;
  }
}

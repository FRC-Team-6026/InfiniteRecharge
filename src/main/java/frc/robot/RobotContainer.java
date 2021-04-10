// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.BarrelRaceCommand;
import frc.robot.commands.BouncePathCommand;
import frc.robot.commands.SampleTrajectoryCommand;
import frc.robot.commands.SlalomCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightController;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.function.*;

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
  private final DoubleSupplier _xBoxSpeedSupplier = () -> filterControllerInputs(-_controller.getY(Hand.kLeft));
  private final DoubleSupplier _xBoxRotationSupplier = () -> filterControllerInputs(_controller.getX(Hand.kRight));
  private final Drive _drive = new Drive(_xBoxSpeedSupplier, _xBoxRotationSupplier);
  private final Shooter _shooter = new Shooter(() -> _controller.getTriggerAxis(Hand.kRight));
  private final Conveyor _conveyor = new Conveyor(() -> _shooter.isAtSetPower());
  private final Intake _intake = new Intake();
  private final LimelightController _limeLight = new LimelightController();
  private final DoubleSupplier _limeLightRotationSupplier = () -> _limeLight.trackTarget();

  private final SampleTrajectoryCommand _sampleTrajectory = new SampleTrajectoryCommand();
  private final BarrelRaceCommand _barrelRaceCommand = new BarrelRaceCommand();
  private final BouncePathCommand _bouncePathCommand = new BouncePathCommand();
  private final SlalomCommand _slalomCommand = new SlalomCommand();

  private final SendableChooser<Command> _autoChooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    _autoChooser.setDefaultOption("zig zag", _sampleTrajectory.getCommand(_drive));
    _autoChooser.addOption("barrel race", _barrelRaceCommand.getCommand(_drive));
    _autoChooser.addOption("bounce path", _bouncePathCommand.getCommand(_drive));
    _autoChooser.addOption("slalom path", _slalomCommand.getCommand(_drive));
    SmartDashboard.putData("Auto mode", _autoChooser);
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
    var startButton = new JoystickButton(_controller, XboxController.Button.kStart.value);

    rightBumper.whenPressed(new InstantCommand(() -> {_intake.run();}, _intake),true)
      .whenReleased(new InstantCommand(() -> {_intake.stop();}, _intake), true);

    leftBumper.whenPressed(new InstantCommand(() -> {_intake.reverse();}, _intake),true)
      .whenReleased(new InstantCommand(() -> {_intake.stop();}, _intake), true);

    aButton.whenPressed(new InstantCommand(() -> {
      _intake.moveArms(!_intake.isExtended());
    }, _intake), true);

    startButton.whenPressed(new InstantCommand(() -> {
      _limeLight.turnLightOn();
      _drive.setRotationSupplier(_limeLightRotationSupplier);
    }), true)
      .whenReleased(new InstantCommand(() -> {
        _limeLight.turnLightOff();
        _drive.setRotationSupplier(_xBoxRotationSupplier);
      }), true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return _autoChooser.getSelected();
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

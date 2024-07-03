// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.math.Filter;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Drive drive = new Drive();
  private Shooter shooter = new Shooter();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick controller =
      new Joystick(OperatorConstants.kDriverControllerPort);

  private final JoystickButton a = new JoystickButton(controller, XboxController.Button.kA.value); // Shooter Button
  private final JoystickButton y = new JoystickButton(controller, XboxController.Button.kY.value); // Tilt Up Button
  private final JoystickButton b = new JoystickButton(controller, XboxController.Button.kB.value); // Tilt Down Button

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    drive.setDefaultCommand(new TeleopDrive(
      drive,
      () -> -controller.getRawAxis(XboxController.Axis.kLeftY.value),
      () -> Filter.powerCurve(controller.getRawAxis(XboxController.Axis.kRightX.value), 3)
    ));
  }


  private void configureBindings() {
    // Runs the shooter
    b.onTrue(new InstantCommand(() -> shooter.shoot(.8, 0)));
    b.onFalse(new InstantCommand(() -> shooter.shoot(0, 0)));

    y.onTrue(new InstantCommand(() -> shooter.tilt(0.1)));
    y.onFalse(new InstantCommand(() -> shooter.tilt(0)));
    
    a.onTrue(new InstantCommand(() -> shooter.tilt(-0.1)));
    a.onFalse(new InstantCommand(() -> shooter.tilt(0)));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}

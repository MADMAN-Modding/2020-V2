// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.math.Filter;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Trigger a = m_driverController.a(); // Shooter Button
  private final Trigger y = m_driverController.y(); // Tilt Up Button
  private final Trigger b = m_driverController.b(); // Tilt Down Button

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    drive.setDefaultCommand(new TeleopDrive(
      drive,
      () -> -m_driverController.getLeftY(),
      () -> Filter.powerCurve(m_driverController.getRightX(), 3)
    ));
  }


  private void configureBindings() {
    // Runs the shooter
    b.onTrue(new InstantCommand(() -> shooter.shoot(1, 1)));
    b.onFalse(new InstantCommand(() -> shooter.shoot(0, 0)));

    y.onTrue(new InstantCommand(() -> shooter.tilt(0.1)));

    a.onTrue(new InstantCommand(() -> shooter.tilt(-0.1)));

  }

  public Command getAutonomousCommand() {
    return null;
  }
}

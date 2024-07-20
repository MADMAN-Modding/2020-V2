// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.math.Filter;
import frc.robot.subsystems.Conveyor;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ConveyorControl;
import frc.robot.commands.MaintainAll;
import frc.robot.commands.ShooterControl;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Drive drive = new Drive();
  private Shooter shooter = new Shooter();
  private Conveyor conveyor = new Conveyor();
  private PhotonVision photonVision = new PhotonVision();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick controller = new Joystick(OperatorConstants.kDriverControllerPort);

  private final JoystickButton a = new JoystickButton(controller, XboxController.Button.kA.value); // Shooter Button
  private final JoystickButton y = new JoystickButton(controller, XboxController.Button.kY.value); // Tilt Up Button
  private final JoystickButton b = new JoystickButton(controller, XboxController.Button.kB.value); // Tilt Down Button

  private final DigitalInput topSwitch = new DigitalInput(Constants.LimitSwitches.topSwitch);
  private final DigitalInput middleSwitch = new DigitalInput(Constants.LimitSwitches.middleSwitch);
  private final DigitalInput bottomSwitch = new DigitalInput(Constants.LimitSwitches.bottomSwitch);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    drive.setDefaultCommand(new TeleopDrive(
        drive,
        () -> -controller.getRawAxis(XboxController.Axis.kLeftY.value),
        () -> Filter.powerCurve(controller.getRawAxis(XboxController.Axis.kRightX.value), 3)));

    shooter.setDefaultCommand(new ShooterControl(() -> topSwitch.get(), () -> b.getAsBoolean(), shooter));
    conveyor.setDefaultCommand(new ConveyorControl(() -> b.getAsBoolean(), () -> topSwitch.get(), conveyor, () -> 0.5));

    // shooter.setDefaultCommand(new ShooterControl(shooter, () -> topSwitch.get()));
  }

  AnalogTrigger input = new AnalogTrigger(1);

  private void configureBindings() {
    // // // Runs the shooter
    // b.onTrue(new InstantCommand(() -> shooter.conveyor(0.7)));
    // b.onFalse(new InstantCommand(() -> shooter.conveyor(0)));
    // b.onTrue(new ConveyorControl(() -> topSwitch.get(), shooter));
    // b.onFalse(new ConveyorControl(() -> false, shooter));

    y.onTrue(new InstantCommand(() -> shooter.tilt(0.1)));
    y.onFalse(new InstantCommand(() -> shooter.tilt(0)));

    a.onTrue(new InstantCommand(() -> shooter.tilt(-0.1)));
    a.onFalse(new InstantCommand(() -> shooter.tilt(0)));
  }

  public void diagnostics() {
    SmartDashboard.putNumber("Left Joystick Y",
        -Filter.deadband(controller.getRawAxis(XboxController.Axis.kLeftY.value), 0.05));
    SmartDashboard.putNumber("Right Joystick X",
        Filter.deadband(controller.getRawAxis(XboxController.Axis.kRightX.value), 0.05));

    SmartDashboard.putBoolean("Top Button", topSwitch.get());
    SmartDashboard.putBoolean("Middle Button", middleSwitch.get());
    SmartDashboard.putBoolean("Bottom Button", bottomSwitch.get());
  }

  public Command getAutonomousCommand() {
    return new MaintainAll(photonVision, drive);
  }
}

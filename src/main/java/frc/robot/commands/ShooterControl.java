// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterControl extends InstantCommand {
  Shooter shooter;
  BooleanSupplier switchPressed;

  public ShooterControl(BooleanSupplier switchPressed, Shooter shooter) {
    this.switchPressed = switchPressed;
    this.shooter = shooter;
    // this.topPressed = topButton;
    addRequirements(shooter);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public void execute() {
    double speed = !switchPressed.getAsBoolean() ? 0.5 : 0;
    shooter.shoot(speed);
  }
}

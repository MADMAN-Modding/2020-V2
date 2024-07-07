// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyor;

public class ConveyorControl extends Command {

  BooleanSupplier buttonPressed;
  BooleanSupplier switchPressed;
  Conveyor conveyor;
  DoubleSupplier speed;

  /** Creates a new ConveyorControl. */
  public ConveyorControl(BooleanSupplier buttonPressed, BooleanSupplier switchPressed, Conveyor conveyor,
      DoubleSupplier speed) {
    this.buttonPressed = buttonPressed;
    this.switchPressed = switchPressed;
    this.conveyor = conveyor;
    this.speed = speed;
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DoubleSupplier speed = buttonPressed.getAsBoolean() && switchPressed.getAsBoolean() ? this.speed : () -> 0;

    conveyor.move(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

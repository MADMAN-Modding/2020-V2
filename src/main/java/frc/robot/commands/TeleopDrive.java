// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class TeleopDrive extends Command {

  public DoubleSupplier leftY;

  public DoubleSupplier rightX;

  public Drive drive;

  /** Creates a new TeleopDrive. */
  public TeleopDrive(Drive drive, DoubleSupplier leftY, DoubleSupplier rightX) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.leftY = leftY;
    this.rightX = rightX;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.drive(leftY.getAsDouble() * Constants.DriveValues.maxSpeed, rightX.getAsDouble() * Constants.DriveValues.maxSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

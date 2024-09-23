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
  BooleanSupplier buttonPressed;

  public ShooterControl(BooleanSupplier switchPressed, BooleanSupplier buttonPressed, Shooter shooter) {
    this.switchPressed = switchPressed;
    this.buttonPressed = buttonPressed;
    this.shooter = shooter;
    // this.topPressed = topButton;
    addRequirements(shooter);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public void execute() {
    double shooterSpeed = !switchPressed.getAsBoolean() && buttonPressed.getAsBoolean() ? 0.5 : 0;
    System.out.println(buttonPressed.getAsBoolean());
    shooter.shoot(shooterSpeed);

    double conveyorSpeed = (switchPressed.getAsBoolean() && buttonPressed.getAsBoolean()) || shooter.getShooterVelocity() > 34000 ? 0.5 : 0;
    shooter.moveConveyor(conveyorSpeed);
  }
}

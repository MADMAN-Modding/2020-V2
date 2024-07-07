// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  private final VictorSPX conveyor = new VictorSPX(Constants.Conveyor.conveyor);
  private final TalonFX beaterBar = new TalonFX(Constants.Conveyor.beaterBar);

  /** Creates a new Conveyor. */
  public Conveyor() {}

  public void move(double conveyorPower) {    
      conveyor.set(VictorSPXControlMode.PercentOutput, conveyorPower);
      beaterBar.set(conveyorPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final VictorSPX conveyor = new VictorSPX(Constants.Conveyor.conveyor);
  private final TalonFX beaterBar = new TalonFX(Constants.Conveyor.beaterBar);
  private final TalonSRX shooterLeft = new TalonSRX(Constants.Shooter.shooterLeft);
  private final TalonSRX shooterRight = new TalonSRX(Constants.Shooter.shooterRight);
  private final TalonFX tilt = new TalonFX(Constants.Shooter.tilt);

  /** Creates a new Shooter. */
  public Shooter() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Tilt Position", tilt.getPosition().getValueAsDouble());
  }

  public void shoot(double power, double conveyorPower) {
    shooterLeft.set(TalonSRXControlMode.PercentOutput, power * -1);
    shooterRight.set(TalonSRXControlMode.PercentOutput, power);

    conveyor.set(VictorSPXControlMode.PercentOutput, conveyorPower);
    beaterBar.set(conveyorPower);
  }

  public void tilt(double direction) {
    tilt.setPosition(tilt.getPosition().getValueAsDouble() + direction);
  }
}

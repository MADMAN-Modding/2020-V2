// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


// Shooter left is commented because the motor burnt
public class Shooter extends SubsystemBase {
  private final VictorSPX conveyor = new VictorSPX(Constants.Conveyor.conveyor);
  private final TalonFX beaterBar = new TalonFX(Constants.Conveyor.beaterBar);
  private final TalonSRX shooterLeft = new TalonSRX(Constants.Shooter.Propulsion.shooterLeft);
  private final TalonSRX shooterRight = new TalonSRX(Constants.Shooter.Propulsion.shooterRight);
  private final TalonFX tilt = new TalonFX(Constants.Shooter.Tilt.tilt);

  /** Creates a new Shooter. */
  public Shooter() {
    // Tilt configuration
    TalonFXConfiguration tiltConfiguration = new TalonFXConfiguration();
    tiltConfiguration.Slot0.kP = Constants.Shooter.Tilt.kP;
    tiltConfiguration.Slot0.kI = Constants.Shooter.Tilt.kI;
    tiltConfiguration.Slot0.kD = Constants.Shooter.Tilt.kD;

    tilt.getConfigurator().apply(new TalonFXConfiguration());

    tilt.getConfigurator().apply(tiltConfiguration.Slot0);

    tilt.setNeutralMode(NeutralModeValue.Brake);

    // shooterLeft.config_kP(0, Constants.Shooter.Propulsion.kP);
    // shooterLeft.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, Constants.Shooter.Propulsion.feedBackSensor, 100);

    // shooterRight.config_kP(0, Constants.Shooter.Propulsion.kP);
    shooterRight.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, Constants.Shooter.Propulsion.feedBackSensor, 100);
 }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Tilt Position", tilt.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Vel", shooterRight.getSelectedSensorVelocity());
  }

  public void shoot(double power, double conveyorPower) {
    shooterLeft.set(TalonSRXControlMode.PercentOutput, power);
    shooterRight.set(TalonSRXControlMode.PercentOutput, power * -1);
    
    SmartDashboard.putNumber("Shooter Commanded Vel", power * -1);

    conveyor.set(VictorSPXControlMode.PercentOutput, conveyorPower);
    beaterBar.set(conveyorPower);
  }


  public void tilt(double direction) {
    tilt.setControl(new DutyCycleOut(direction));
  }
}

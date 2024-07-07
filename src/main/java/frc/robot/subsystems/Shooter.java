// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// Shooter left is commented because the motor burnt
public class Shooter extends SubsystemBase {
  private final TalonSRX shooterLeft = new TalonSRX(Constants.Shooter.Propulsion.shooterLeft);
  private final TalonSRX shooterRight = new TalonSRX(Constants.Shooter.Propulsion.shooterRight);
  private final TalonFX tilt = new TalonFX(Constants.Shooter.Tilt.tilt);

  public DoubleSupplier conveyorSpeed = () -> 0.5;

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

    shooterRight.config_kP(0, Constants.Shooter.Propulsion.kP);
    shooterRight.config_kI(0, Constants.Shooter.Propulsion.kI);
    shooterRight.config_kD(0, Constants.Shooter.Propulsion.kD);
    shooterRight.config_kF(0, Constants.Shooter.Propulsion.kF);
    shooterRight.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder,
        Constants.Shooter.Propulsion.feedBackSensor, 100);
    shooterRight.setInverted(Constants.Shooter.Propulsion.shooterRightInverted);
    shooterLeft.setInverted(Constants.Shooter.Propulsion.shooterLeftInverted);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Tilt Position", tilt.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Vel", shooterRight.getSelectedSensorVelocity());
  }

  public void shoot(double shooterPower) {
    shooterRight.set(TalonSRXControlMode.PercentOutput, shooterPower);
    shooterLeft.follow(shooterRight);
  }



  public void tilt(double direction) {
    tilt.setControl(new DutyCycleOut(direction));
  }
}

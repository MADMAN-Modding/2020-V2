// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Filter;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */
  private final TalonSRX driveLeft = new TalonSRX(Constants.DriveValues.driveLeft);
  private final TalonSRX driveRight = new TalonSRX(Constants.DriveValues.driveRight);
  private final TalonSRX followerLeft = new TalonSRX(Constants.DriveValues.followerLeft);
  private final TalonSRX followerRight = new TalonSRX(Constants.DriveValues.followerRight);

  private double increment = 0;

  public Drive() {
    driveLeft.setInverted(Constants.DriveValues.driveLeftInverted);
    driveRight.setInverted(Constants.DriveValues.driveRightInverted);
    followerLeft.setInverted(Constants.DriveValues.followerLeftInverted);
    followerRight.setInverted(Constants.DriveValues.followerRightInverted);

    driveLeft.config_kP(Constants.DriveValues.leftDrivePID.leftMotorPIDController, Constants.DriveValues.leftDrivePID.kP);
    driveRight.config_kP(Constants.DriveValues.rightDrivePID.rightMotorPIDController, Constants.DriveValues.rightDrivePID.kP);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Drive Encoder", driveLeft.getSelectedSensorPosition(Constants.DriveValues.leftDrivePID.leftMotorPIDController));
    SmartDashboard.putNumber("Right Drive Encoder", driveRight.getSelectedSensorPosition(Constants.DriveValues.rightDrivePID.rightMotorPIDController));
  }

  public void drive(double left, double right) {

    left = Filter.deadband(left, 0.04);
    right = Filter.deadband(right, 0.04);

    double leftSpeed = Filter.cutoffFilter(left+right);

    double rightSpeed = Filter.cutoffFilter(left-right);

    driveLeft.set(TalonSRXControlMode.PercentOutput, leftSpeed);
    driveRight.set(TalonSRXControlMode.PercentOutput, rightSpeed);

    followerLeft.follow(driveLeft);
    followerRight.follow(driveRight);
  }

  public void updatePID(double increment) {

    this.increment += increment;

    driveLeft.config_kP(Constants.DriveValues.leftDrivePID.leftMotorPIDController, Constants.DriveValues.leftDrivePID.kP + this.increment);
    driveRight.config_kP(Constants.DriveValues.rightDrivePID.rightMotorPIDController, Constants.DriveValues.rightDrivePID.kP + this.increment);
  }
}

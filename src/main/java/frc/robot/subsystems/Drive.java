// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Filter;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */
  private final TalonSRX driveLeft = new TalonSRX(Constants.DriveValues.driveLeft);
  private final TalonSRX driveRight = new TalonSRX(Constants.DriveValues.driveRight);
  private final VictorSPX followerLeft = new VictorSPX(Constants.DriveValues.followerLeft);
  private final VictorSPX followerRight = new VictorSPX(Constants.DriveValues.followerRight);

  public Drive() {
    driveLeft.setInverted(Constants.DriveValues.driveLeftInverted);
    driveRight.setInverted(Constants.DriveValues.driveRightInverted);
    followerLeft.setInverted(Constants.DriveValues.followerLeftInverted);
    followerRight.setInverted(Constants.DriveValues.followerRightInverted);

    driveLeft.config_kP(Constants.DriveValues.leftDrivePID.leftMotorPIDController, Constants.DriveValues.leftDrivePID.kP);
    driveRight.config_kP(Constants.DriveValues.rightDrivePID.rightMotorPIDController, Constants.DriveValues.rightDrivePID.kP);

    driveLeft.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, Constants.DriveValues.leftDrivePID.leftMotorPIDController, 100);
    driveRight.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, Constants.DriveValues.rightDrivePID.rightMotorPIDController, 100);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Drive Encoder", driveLeft.getSelectedSensorPosition(Constants.DriveValues.leftDrivePID.leftMotorPIDController));
    SmartDashboard.putNumber("Right Drive Encoder", driveRight.getSelectedSensorPosition(Constants.DriveValues.rightDrivePID.rightMotorPIDController));
    SmartDashboard.putNumber("Left Sensor Velocity", driveLeft.getSelectedSensorVelocity(Constants.DriveValues.leftDrivePID.leftMotorPIDController));
    SmartDashboard.putNumber("Right Sensor Velocity", driveRight.getSelectedSensorVelocity(Constants.DriveValues.leftDrivePID.leftMotorPIDController));
  }

  public void drive(double left, double right) {

    left = Filter.deadband(left, 0.04);
    right = Filter.deadband(right, 0.04);

    double leftSpeed = Filter.cutoffFilter(left+right);

    double rightSpeed = Filter.cutoffFilter(left-right);

    driveLeft.set(ControlMode.Velocity, leftSpeed * 1000);
    driveRight.set(ControlMode.Velocity, rightSpeed * 1000);

    followerLeft.follow(driveLeft);
    followerRight.follow(driveRight);
  }
}

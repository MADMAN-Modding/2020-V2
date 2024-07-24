// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Filter;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */
  private final TalonSRX driveLeft = new TalonSRX(Constants.DriveValues.driveLeftID);
  private final TalonSRX driveRight = new TalonSRX(Constants.DriveValues.driveRightID);
  private final VictorSPX followerLeft = new VictorSPX(Constants.DriveValues.followerLeft);
  private final VictorSPX followerRight = new VictorSPX(Constants.DriveValues.followerRight);
  
  Pose2d pose2d = new Pose2d(0, 0, new Rotation2d(0));

  public Drive() {
    // Setting configs to avoid weird stuff from happening with configs, yeah, wahoo
    driveLeft.configFactoryDefault();
    followerLeft.configFactoryDefault();

    driveRight.configFactoryDefault();
    followerRight.configFactoryDefault();

    driveLeft.setInverted(Constants.DriveValues.driveLeftInverted);
    driveRight.setInverted(Constants.DriveValues.driveRightInverted);
    followerLeft.setInverted(Constants.DriveValues.followerLeftInverted);
    followerRight.setInverted(Constants.DriveValues.followerRightInverted);

    driveLeft.config_kP(Constants.DriveValues.driveLeftPID.leftMotorPIDController, Constants.DriveValues.driveLeftPID.kP);
    driveLeft.config_kI(Constants.DriveValues.driveLeftPID.leftMotorPIDController, Constants.DriveValues.driveLeftPID.kI);
    driveLeft.config_kD(Constants.DriveValues.driveLeftPID.leftMotorPIDController, Constants.DriveValues.driveLeftPID.kD);
    driveLeft.config_kF(Constants.DriveValues.driveLeftPID.leftMotorPIDController, Constants.DriveValues.driveLeftPID.kF);

    driveRight.config_kP(Constants.DriveValues.driveRightPID.rightMotorPIDController, Constants.DriveValues.driveRightPID.kP);
    driveRight.config_kI(Constants.DriveValues.driveRightPID.rightMotorPIDController, Constants.DriveValues.driveRightPID.kI);
    driveRight.config_kF(Constants.DriveValues.driveRightPID.rightMotorPIDController, Constants.DriveValues.driveRightPID.kD);  
    driveRight.config_kD(Constants.DriveValues.driveRightPID.rightMotorPIDController, Constants.DriveValues.driveRightPID.kF);

    driveLeft.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, Constants.DriveValues.driveLeftPID.leftMotorPIDController, 100);
    driveRight.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, Constants.DriveValues.driveRightPID.rightMotorPIDController, 100);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Drive Encoder", driveLeft.getSelectedSensorPosition(Constants.DriveValues.driveLeftPID.leftMotorPIDController));
    SmartDashboard.putNumber("Right Drive Encoder", driveRight.getSelectedSensorPosition(Constants.DriveValues.driveRightPID.rightMotorPIDController));
    SmartDashboard.putNumber("Left Sensor Velocity", driveLeft.getSelectedSensorVelocity(Constants.DriveValues.driveLeftPID.leftMotorPIDController) * -1);
    SmartDashboard.putNumber("Right Sensor Velocity", driveRight.getSelectedSensorVelocity(Constants.DriveValues.driveRightPID.rightMotorPIDController) * -1);
  }

  public void drive(double left, double right) {

    left = Filter.deadband(left, 0.05);
    right = Filter.deadband(right, 0.05);

    double leftSpeed = Filter.cutoffFilter(left+right);

    double rightSpeed = Filter.cutoffFilter(left-right);

    SmartDashboard.putNumber("Commanded Velocity Left", leftSpeed * 1000);
    SmartDashboard.putNumber("Commanded Velocity Right", rightSpeed * 1000);

    driveLeft.set(ControlMode.Velocity, leftSpeed * 1000);
    driveRight.set(ControlMode.Velocity, rightSpeed * 1000);

    // driveLeft.set(ControlMode.PercentOutput, leftSpeed * 1.25);
    // driveRight.set(ControlMode.PercentOutput, rightSpeed);

    followerLeft.follow(driveLeft);
    followerRight.follow(driveRight);

    // followerLeft.set(ControlMode.PercentOutput, leftSpeed);
    // followerRight.set(ControlMode.PercentOutput, rightSpeed);
  }
}

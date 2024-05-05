// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Filter;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */
  private final TalonSRX driveLeft = new TalonSRX(Constants.DriveValues.driveLeft);
  private final TalonSRX driveRight = new TalonSRX(Constants.DriveValues.driveRight);
  private final TalonSRX followerLeft = new TalonSRX(Constants.DriveValues.followerLeft);
  private final TalonSRX followerRight = new TalonSRX(Constants.DriveValues.followerRight);


  public Drive() {
    driveLeft.setInverted(Constants.DriveValues.driveLeftInverted);
    driveRight.setInverted(Constants.DriveValues.driveRightInverted);
    followerLeft.setInverted(Constants.DriveValues.followerLeftInverted);
    followerRight.setInverted(Constants.DriveValues.followerRightInverted);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double left, double right) {

    left = Filter.deadband(left, 0.04);
    right = Filter.deadband(right, 0.04);

    double leftSpeed = Filter.cutoffFilter(left+right);

    double rightSpeed = Filter.cutoffFilter(left-right);

    driveLeft.set(TalonSRXControlMode.PercentOutput, leftSpeed);
    driveRight.set(TalonSRXControlMode.PercentOutput, rightSpeed);

    followerLeft.set(TalonSRXControlMode.PercentOutput, leftSpeed);
    followerRight.set(TalonSRXControlMode.PercentOutput, rightSpeed);
  }
}

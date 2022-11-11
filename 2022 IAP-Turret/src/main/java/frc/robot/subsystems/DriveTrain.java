// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;
  
  public DriveTrain() {
leftDriveTalon = new WPI_TalonSRX(Constants.leftTalon);
rightDriveTalon = new WPI_TalonSRX(Constants.rightTalon);
leftDriveTalon.setInverted(false);
rightDriveTalon.setInverted(true);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    rightDriveTalon.set(rightSpeed);
    leftDriveTalon.set(leftSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}

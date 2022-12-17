// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;
  private final AHRS navx = new AHRS(SPI.Port.kMXP);
  
  public DriveTrain() {
leftDriveTalon = new WPI_TalonSRX(Constants.leftTalon);
rightDriveTalon = new WPI_TalonSRX(Constants.rightTalon);
leftDriveTalon.setInverted(false);
rightDriveTalon.setInverted(true);
  }

  public double getAngle(){
    return navx.getAngle();
  }

  public void reset(){
    navx.reset();
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    rightDriveTalon.set(rightSpeed);
    leftDriveTalon.set(leftSpeed);
  }

  @Override
  public void periodic() {
    tankDrive(RobotContainer.GetJoy1().getY()*0.2, RobotContainer.GetJoy2().getY()*0.2); 
    //Giving controls to the joysticks
    SmartDashboard.putNumber("leftspeed", leftDriveTalon.get());
    SmartDashboard.putNumber("rightspeed", rightDriveTalon.get());
    SmartDashboard.putNumber("Navx angle", navx.getAngle());
    // This method will be called once per scheduler run
  }




}

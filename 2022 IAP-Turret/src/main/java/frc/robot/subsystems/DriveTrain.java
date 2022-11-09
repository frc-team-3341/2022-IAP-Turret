// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase 
{
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  private double DriveToLineDirection = 1.0;
  private double DriveToLineOffset = 0.0;

  private ShuffleboardTab DTTab = Shuffleboard.getTab("DriveTrain");
  private NetworkTableEntry DTDisplacement = DTTab.add("Displacement", 0.0).getEntry();
  private NetworkTableEntry LeftVelocity = DTTab.add("Left Native Velocity", 0.0).getEntry();
  private NetworkTableEntry RightVelocity = DTTab.add("Right Native Velocity", 0.0).getEntry();
  private NetworkTableEntry LeftVoltage = DTTab.add("Left Voltage", 0.0).getEntry();
  private NetworkTableEntry RightVoltage = DTTab.add("Right Voltage", 0.0).getEntry();


  public DriveTrain() 
  {
    leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
    rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);
  
    leftDriveTalon.setNeutralMode(NeutralMode.Coast);
    rightDriveTalon.setNeutralMode(NeutralMode.Coast);

    leftDriveTalon.setInverted(false);
    rightDriveTalon.setInverted(true);

    leftDriveTalon.setSensorPhase(true);
    rightDriveTalon.setSensorPhase(true);

    leftDriveTalon.configFactoryDefault();
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightDriveTalon.configFactoryDefault();
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    //diff.tankDrive(leftSpeed, rightSpeed);
    rightDriveTalon.set(rightSpeed);
    leftDriveTalon.set(leftSpeed);
  }

  public void resetEncoders() {
    leftDriveTalon.setSelectedSensorPosition(0,0,10);
    rightDriveTalon.setSelectedSensorPosition(0,0,10);
  }

  public double getDisplacement() {
    return (getTicks() / (Constants.DriveToLineConstants.ticksToMeters));
  }

  public double getDTLOffset() {
    return DriveToLineOffset;
  }

  public double getDTLDirection() {
    return DriveToLineDirection;
  }

  public double getTicks() {
    return (leftDriveTalon.getSelectedSensorPosition(0) + rightDriveTalon.getSelectedSensorPosition(0)) / 2.0;
  }

  public double getAngleAndReset(){
    double degrees = navx.getAngle();
    navx.reset();
    return degrees;
  }
 
  public double getAngle(){
    return navx.getAngle(); 
  }
 
  public void resetN(){
    navx.reset();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Voltage", leftDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Voltage", rightDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Left Native Velocity", leftDriveTalon.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Native Velocity", leftDriveTalon.getSelectedSensorVelocity());

    DTDisplacement.setDouble(getDisplacement());
    LeftVelocity.setDouble(leftDriveTalon.getSelectedSensorVelocity());
    RightVelocity.setDouble(rightDriveTalon.getSelectedSensorVelocity());
    LeftVoltage.setDouble(leftDriveTalon.getMotorOutputPercent());
    RightVoltage.setDouble(rightDriveTalon.getMotorOutputPercent());
    
    tankDrive(RobotContainer.getJoy1().getY()*-0.2, RobotContainer.getJoy2().getY()*-0.2);
  }
}
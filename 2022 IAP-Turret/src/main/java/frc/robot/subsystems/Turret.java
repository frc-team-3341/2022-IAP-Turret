// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PhotonVision;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends SubsystemBase {
  
  private final WPI_TalonSRX motor;
  private Joystick joy = new Joystick(0);
  private Turret turret = new Turret();
  /** Creates a new Turret. */

  public Turret() {
    motor = new WPI_TalonSRX(Constants.TurretConstants.talonPort);
    motor.configFactoryDefault();
    motor.setInverted(false);
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);

  

    motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
  }
  
  public boolean targetLocked(PhotonVision photonVision){
    return photonVision.targetExists() && Math.abs(photonVision.getYaw()) < Constants.TurretConstants.targetLockedWidth; 
    // This tells you if the target is locked in our area. We are using targetLockedWidth from robot constants. 
    // In the above line we are using photonVision.getYaw because the Yaw is in the PhotonVision object. 
  }

  public void setAngle(double angle){
    motor.setSelectedSensorPosition(angle);
  }

  public double getAngle(){
    return motor.getSelectedSensorPosition() * 360 / (4096.0);
  }
  
  public void spin(double speed)
  {
    motor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Angle: ", getAngle());
    //SmartDashboard.putNumber("Joystick Power: ", RobotContainer.getJoy().getY());
  }

}
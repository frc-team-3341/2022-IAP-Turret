// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;


public class ProtoTurret extends CommandBase {
  /** Creates a new ProtoTurret. */

  public final PhotonVision photon;
  public final DriveTrain dt;
  public PIDController pid;
  public boolean directionToggle;

public ProtoTurret(DriveTrain dt, PhotonVision photon) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.photon = photon;
    this.dt = dt;
    pid = new PIDController(0.01, 0.0, 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.reset();
    pid.setSetpoint(0);
}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //All of this asks the camera if it sees the object and if not then it will turn right or left.
    //If target found then I use PID to get an accurate location and then rotate to the target
    if (!photon.targetExists()) {
      if (directionToggle) {
        dt.tankDrive(-0.2, 0.2);
      }
      if (!directionToggle) {
        dt.tankDrive(0.2, -0.2);
      }
      if (dt.getAngle() >= 100) { 
        // if angle is less than 90 it goes to the right
        directionToggle = true;
      } else if (dt.getAngle() <= -10) { 
        // if angle is less than 10 it goes to the left, this is because you're using an unit circle 
        directionToggle = false;
      } 
  } else if (photon.targetExists()){
    //This using PIDs calculates how far the center is to the target from yaw and then speed is set so it can precisely locate the target
      double speed = pid.calculate(photon.getYaw());
      dt.tankDrive(-speed, speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

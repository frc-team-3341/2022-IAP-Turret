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

public ProtoTurret(DriveTrain dt, PhotonVision photon) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.photon = photon;
    this.dt = dt;
    pid = new PIDController(0.0, 0.0, 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {/*
    if (!photon.targetExists()) {
      if (dt.getAngle() < 90) { 
        // if angle is less than 90 it goes to the right
        dt.tankDrive(-0.2, 0.2);
      } else if (dt.getAngle() > 10) { 
        // if angle is less than 10 it goes to the left
        dt.tankDrive(0.2, -0.2);
      }
  }*/
}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

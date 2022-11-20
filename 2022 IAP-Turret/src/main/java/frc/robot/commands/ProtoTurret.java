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
    pid = new PIDController(0.0, 0.0, 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetN();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!photon.targetExists()) {
      if (dt.getAngle() >= 100) {
        directionToggle = true;
      }
      if (dt.getAngle() <= 10) {
        directionToggle = false;
      }
      if (directionToggle) { // NOTE -  Directions are correct!
        // Mimics the hard-defined stops of the real Turret
        // Goes CW if angle is equal to/greater than 100
        dt.tankDrive(0.3, -0.3);
      } else if (!directionToggle) {
        // Goes CCW if angle is equal to or less than 10
        dt.tankDrive(-0.3, 0.3);
      }
    }
    else if (photon.targetExists()) {
      if (dt.getAngle() < 100 | dt.getAngle() > 10) { // TODO - Test for direction! PID Constants!
        double speed = pid.calculate(photon.getPitch()); // If it exists and within range, apply PID to the output
        dt.tankDrive(-speed, speed);
      } else {
        dt.tankDrive(0.0, 0.0);
      }
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

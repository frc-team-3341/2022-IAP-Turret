// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;

public class PIDTurn extends CommandBase {
  public final PhotonVision photon;
  public final DriveTrain dt;
  public PIDController pid;
  public double setpoint;
  public double calculatedPower;

  /** Creates a new PIDTurn. */
  public PIDTurn(DriveTrain dt, PhotonVision photon, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.photon = photon;
    this.dt = dt;
    this.setpoint = angle;
    // Karen's PID kP constant from last year was 0.7!
    pid = new PIDController(0.1, 0.0, 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetN();
    pid.setSetpoint(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calculatedPower = pid.calculate(dt.getAngle());
    dt.tankDrive(-calculatedPower, calculatedPower); // TODO - figure out sign of power
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (dt.getAngle() >= setpoint);
  }
}

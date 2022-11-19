// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;


public class TurnToAngle extends CommandBase {

  private Turret turret;
  private double angle;
  private Joystick joy;
  private boolean teleOp;
  PIDController pid;

  /** Creates a new TurnToAngle. */
  public TurnToAngle(Turret turret, double angle, Joystick joy, boolean teleOp) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.angle = angle;
    this.joy = joy;
    this.teleOp = teleOp;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PIDController(Constants.PIDConstants.kP, Constants.PIDConstants.kI, Constants.PIDConstants.kD);
    if(!teleOp) pid.setSetpoint(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(teleOp) {
      turret.spin(-0.5 * joy.getY());
      SmartDashboard.putNumber("Joystick Power", joy.getY());
    }
    else turret.spin(pid.calculate(turret.getAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.spin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(angle - turret.getAngle()) < Constants.TurretConstants.threshold;
  }
}
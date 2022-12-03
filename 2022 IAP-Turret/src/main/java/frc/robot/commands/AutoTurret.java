// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Turret;

public class AutoTurret extends CommandBase {
  /** Creates a new AutoTurret. */
private Turret turret;
private PhotonVision photonVision;
  // States for limit switches (open or closed)
  private double LimitSwitchClosed = 0.0; // 0 for closed state 
  private double LimitSwitchOpen = 1; // 1 for open state

  public AutoTurret(Turret turret, PhotonVision photonVision) {
    this.turret = turret; 
    // You don't need to get anything from the object turret because we already have it here. Use turret.wtv to get stuff
    this.photonVision = photonVision; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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

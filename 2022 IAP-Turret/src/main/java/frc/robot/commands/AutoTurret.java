// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Turret;

public class AutoTurret extends CommandBase {
  /** Creates a new AutoTurret. */
  public final PhotonVision photon;
  public final Turret turret;
  public PIDController pid;
  public boolean direction;
  // States for limit switches (open or closed)
  // 1 for the switch is not pressed so it's normally closed
  // 0 for the switch is pressed
  private double LimitSwitchClosed = 0.0; // 0 for closed state 

  public AutoTurret(Turret turret, PhotonVision photon) {
    this.turret = turret; 
    // You don't need to get anything from the object turret because we already have it here. Use turret.wtv to get stuff
    this.photon = photon; 
    pid = new PIDController(0.01142, 0.0, 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.resetEncoders();
    pid.setSetpoint(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!photon.targetExists()) {

// We are having limitswitchclosed because it is a closed circuit

      if (turret.getCCW_Reverse_LimitSw() == LimitSwitchClosed) {
         // True means the limit switch is hit
         // in the reverse direction and will go CW
        direction = true;
      }

      if (turret.getCW_Forward_LimitSw() == LimitSwitchClosed) {
         // False means that it hit the limit switch
         // in the forward direction and will go CCW
        direction = false;
      }

      if (direction) { // NOTE - need to test direction
        // Goes CW if reverse limit switch is hit
        turret.spin(0.1);
        
      } else if (!direction) { // NOTE - need to test direction
        // Goes CCW if forward limit switch is hit
        turret.spin(-0.1);

      } else if(photon.targetExists() && (turret.getCCW_Reverse_LimitSw() == LimitSwitchClosed && turret.getCW_Forward_LimitSw() == LimitSwitchClosed)) {
        double speed = pid.calculate(photon.getYaw()); // If it target exists and is within range it will apply PID to the output
        turret.spin(speed); // Spins clockwise for positive yaw value
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


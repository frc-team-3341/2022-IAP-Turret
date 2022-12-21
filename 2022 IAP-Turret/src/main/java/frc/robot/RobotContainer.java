// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoTurret;
import frc.robot.commands.ProtoTurret;
import frc.robot.commands.SpinToTarget;
// import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final PhotonVision photon;
  //private final DriveTrain dt;

  public static Joystick joystick1;
  public static Joystick joystick2;

  //private final SpinToTarget spin;
  //private final ProtoTurret proto;
  private final AutoTurret auto;
  private final Turret turret;

  // The robot's subsystems and commands are defined here...
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    photon = new PhotonVision();

    joystick1 = new Joystick(Constants.USBOrder.Zero);
    joystick2 = new Joystick(Constants.USBOrder.One);
    //dt = new DriveTrain();
    turret = new Turret();
    turret.resetEncoders();

    //spin = new SpinToTarget(dt, photon);
    //proto = new ProtoTurret(dt, photon);
    auto = new AutoTurret(photon, turret);
    turret.setDefaultCommand(auto);

    // Configure the button bindings
    configureButtonBindings();
  }

  public static Joystick getJoy1() {
    return joystick1;
  }

  public static Joystick getJoy2() {
    return joystick2;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}

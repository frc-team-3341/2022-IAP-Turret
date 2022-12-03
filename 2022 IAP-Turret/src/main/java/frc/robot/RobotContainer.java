// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Turret;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private Turret turret;
  private TurnToAngle teleOp;
  private TurnToAngle turn0;
  private TurnToAngle turn30;
  private TurnToAngle turn45;
  private TurnToAngle turn60;
  private TurnToAngle turn75;
  private TurnToAngle turn90;
  private static Joystick joy;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    joy = new Joystick(0);
    turret = new Turret();

    turret.setAngle(0);

    teleOp = new TurnToAngle(turret, 0, joy, true);
    turn0 = new TurnToAngle(turret, 0, joy, false);
    turn30 = new TurnToAngle(turret, 30.0, joy, false);
    turn45 = new TurnToAngle(turret, 45.0, joy, false);
    turn60 = new TurnToAngle(turret, 60.0, joy, false);
    turn75 = new TurnToAngle(turret, 75.0, joy, false);
    turn90 = new TurnToAngle(turret, 90.0, joy, false);

    turret.setDefaultCommand(teleOp);
    configureButtonBindings();
  }

  public static Joystick getJoy(){
    return joy;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton switchMode = new JoystickButton(joy, 1);
    JoystickButton button30 = new JoystickButton(joy, 7);
    JoystickButton button45 = new JoystickButton(joy, 8);
    JoystickButton button60 = new JoystickButton(joy, 9);
    JoystickButton button75 = new JoystickButton(joy, 10);
    JoystickButton button90 = new JoystickButton(joy, 11);
    JoystickButton button0 = new JoystickButton(joy, 12);

    switchMode.whenPressed(teleOp);
    button30.whenPressed(turn30);
    button45.whenPressed(turn45);
    button60.whenPressed(turn60);
    button75.whenPressed(turn75);
    button90.whenPressed(turn90);
    button0.whenPressed(turn0);

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}

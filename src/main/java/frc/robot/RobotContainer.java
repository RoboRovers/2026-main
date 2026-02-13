// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project..

package frc.robot;

import frc.robot.Subsystems.Drive.Swerve;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Commands.Drive;
import frc.robot.Util.Constants;
import frc.robot.Util.Controllers;
import frc.robot.Util.RobotMap.MAP_CONTROLLER;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public Controllers u_Controllers;
  public Swerve s_Swerve;
  public Drive c_Drive;
  public Intake s_Intake;
  public Shooter s_Shooter;
  
  // public Auto c_Auto;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings 
    robotFiles();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void robotFiles() {
    u_Controllers = new Controllers();
    s_Swerve = new Swerve();
    c_Drive = new Drive(s_Swerve, u_Controllers.leftStick, u_Controllers.rightStick);
    s_Shooter = new Shooter();
  }
  private void configureBindings() {
    //Intake Bindings
    u_Controllers.xbox.leftTrigger().whileTrue(s_Intake.spinRollers());
    u_Controllers.xbox.rightTrigger().whileTrue(s_Shooter.shootFuel());
    u_Controllers.xbox.y().whileTrue(s_Intake.intakeIn());
    u_Controllers.xbox.x().whileTrue(s_Intake.intakeOut());
    
    //Drive Bindings
    u_Controllers.rightStick.button(2).toggleOnTrue(Commands.runOnce(() -> s_Swerve.zeroHeading()));
    u_Controllers.rightStick.button(3).toggleOnTrue(s_Swerve.fieldOrientedToggle());
    u_Controllers.rightStick.button(4).onTrue(s_Swerve.resetWheels()); //window looking button
    // No-op example bindings removed. Add controller bindings here.

    //Shooter Bindings
    u_Controllers.shootFuel.whileTrue(s_Shooter.shootFuel());
    u_Controllers.shootFuel.whileFalse(s_Shooter.stop());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}

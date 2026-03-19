// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.ExampleSubsystem;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Drive.Swerve;
import frc.robot.Util.Constants.Constants_AprilTags;
import frc.robot.Util.Constants.Constants_Auto;

public final class Autos {
  // Only the things we actually need to reference later
  private final Swerve s_Swerve;
  private RobotConfig config;
  private final PathFollowingController pathController;

  public Autos(Swerve swerve, Shooter s_Shooter, RobotConfig defaultConfig) {
    this.s_Swerve = swerve;

    // These stay local to the constructor
    // Use the constant values directly instead of constructing PIDController instances
    try {
      this.config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      this.config = defaultConfig;
      e.printStackTrace();
    }

    this.pathController = new PPHolonomicDriveController(
        new PIDConstants(Constants_Auto.P_TRANSLATION, Constants_Auto.I_TRANSLATION, Constants_Auto.D_TRANSLATION),
        new PIDConstants(Constants_Auto.P_THETA, Constants_Auto.I_THETA, Constants_Auto.D_THETA)
    );

    AutoBuilder.configure(
      this.s_Swerve::getPose,
      this.s_Swerve::resetOdometry,
      this.s_Swerve::getRobotRelativeSpeeds,
      this.s_Swerve::setModuleStates,
      pathController,
      this.config,
      this.s_Swerve::allianceCheck,
      this.s_Swerve);

    // NamedCommands "capture" the subsystems they need
    NamedCommands.registerCommand("Face Forward Wheels", Commands.runOnce(this.s_Swerve::faceAllForward));
    NamedCommands.registerCommand("Shoot", Commands.runOnce(s_Shooter::remoteShootFuel));
  } 
}


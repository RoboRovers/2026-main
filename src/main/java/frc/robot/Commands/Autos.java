// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
  private Swerve s_Swerve;
  //private Vision s_Vision;//TODO: Add vision subsystem
  private RobotConfig config;
  private PIDController translationConstants = new PIDController(Constants_Auto.P_TRANSLATION, Constants_Auto.I_TRANSLATION, Constants_Auto.D_TRANSLATION);
  private PIDController rotationConstants = new PIDController(Constants_Auto.P_THETA, Constants_Auto.I_THETA, Constants_Auto.D_THETA);
  
  public Autos(Drive c_Drive, Swerve s_Swerve, Shooter s_Shooter, Intake s_Intake/* , Vision s_Vision*/, RobotConfig config) {
    this.s_Swerve = s_Swerve;
    //this.s_Vision = s_Vision;//TODO: Add vision subsystem
 

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }
  
    AutoBuilder.configure(
      s_Swerve::getPose,
      s_Swerve::resetOdometry,
      s_Swerve::getRobotRelativeSpeeds,
      s_Swerve::setModuleStates,
      pathController,
      config,
      s_Swerve::allianceCheck,
      s_Swerve);


      NamedCommands.registerCommand("Face Forward Wheels", Commands.runOnce(() -> s_Swerve.faceAllForward())); //Should be ran at the start of every auto.
      }
      
    public PathFollowingController pathController = new PPHolonomicDriveController(
      new com.pathplanner.lib.config.PIDConstants(translationConstants.getP(), translationConstants.getI(), translationConstants.getD()),
      new com.pathplanner.lib.config.PIDConstants(rotationConstants.getP(), rotationConstants.getI(), rotationConstants.getD()));

}



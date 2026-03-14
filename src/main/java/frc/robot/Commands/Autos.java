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
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drive.Swerve;
import frc.robot.Util.Constants.Constants_Auto;

public final class Autos extends Command {
  // Only the things we actually need to reference later
  private final Swerve s_Swerve;
  private final Climber s_Climb;
  private RobotConfig config;
  private final PathFollowingController pathController;

  public Autos(Swerve s_Swerve, Climber s_Climb, RobotConfig defaultConfig) {
    this.s_Swerve = s_Swerve;
    this.s_Climb = s_Climb;

    // These stay local to the constructor
    PIDController translationPID = new PIDController(Constants_Auto.P_TRANSLATION, Constants_Auto.I_TRANSLATION, Constants_Auto.D_TRANSLATION);
    PIDController rotationPID = new PIDController(Constants_Auto.P_THETA, Constants_Auto.I_THETA, Constants_Auto.D_THETA);

    try {
      this.config = RobotConfig.fromGUISettings();
    } 
    catch (Exception e) {
      this.config = defaultConfig;
      e.printStackTrace();
    }

    this.pathController = new PPHolonomicDriveController(
        new PIDConstants(translationPID.getP(), translationPID.getI(), translationPID.getD()),
        new PIDConstants(rotationPID.getP(), rotationPID.getI(), rotationPID.getD())
    );

    AutoBuilder.configure(
      s_Swerve::getPose,
      s_Swerve::resetOdometry,
      s_Swerve::getRobotRelativeSpeeds,
      s_Swerve::setModuleStates,
      pathController,
      this.config,
      s_Swerve::allianceCheck,
      s_Swerve);

    // NamedCommands "capture" the subsystems they need
    NamedCommands.registerCommand("Face Forward Wheels", Commands.runOnce(s_Swerve::faceAllForward));
  } 
      @Override
    public void initialize()
    {

    }
    @Override
    public void execute()
    {

    }
    @Override
    public void end(boolean interrupted)
    {

    }
    
}


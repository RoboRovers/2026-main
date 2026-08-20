// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Util.Constants.Constants_Auto;
import frc.robot.Subsystems.Drive.Swerve;

public final class Autos {
    private RobotConfig config;
    private PIDController transConstants = new PIDController(Constants_Auto.P_TRANSLATION, Constants_Auto.I_TRANSLATION, Constants_Auto.D_TRANSLATION);
    private PIDController rotConstants = new PIDController(Constants_Auto.P_THETA, Constants_Auto.I_THETA, Constants_Auto.D_THETA);

    public Autos(Drive s_Drive, Swerve s_Swerve) {
        try 
        {
            this.config = RobotConfig.fromGUISettings();
        }  
        catch (Exception e) 
        {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            s_Swerve::getPose, 
            s_Swerve::resetOdometry, 
            s_Swerve::getRobotRelativeSpeeds,
            s_Swerve::setModuleStates,
            pathController,
            this.config,
            s_Swerve::allianceCheck,
            s_Swerve);

        NamedCommands.registerCommand("Face Forward Wheels", Commands.runOnce(s_Swerve::faceAllForward));
        NamedCommands.registerCommand("Face Right Wheels", Commands.runOnce(s_Swerve::faceAllRight));
    }

    public PathFollowingController pathController = new PPHolonomicDriveController(
        new PIDConstants(transConstants.getP(), transConstants.getI(), transConstants.getD()),
        new PIDConstants(rotConstants.getP(), rotConstants.getI(), rotConstants.getD()) 
    );
}

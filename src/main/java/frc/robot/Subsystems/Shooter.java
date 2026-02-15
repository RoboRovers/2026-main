// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import frc.robot.Util.Constants;
import frc.robot.Util.Constants.Constants_Shooter;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Util.Constants.FuelConstants.*;

public class Shooter extends SubsystemBase {
  private final SparkMax shooterRoller;
  public final SparkMax fuelAgitator;

  /** Creates a new Shooter Subsystem. */
  public Shooter() {
    // create brushed motors for each of the motors on the shooter mechanism
    shooterRoller = new SparkMax(Constants_Shooter.shooterMotorID, MotorType.kBrushed);
    fuelAgitator = new SparkMax(Constants_Shooter.fuelAgitatorMotorID, MotorType.kBrushed);
  
    // create the configuration for the shooter roller, set a current limit, (set
    // the motor to inverted so that positive values are used for shooting???), 
    // and apply the config to the controller
    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    // TODO: Not sure if the motor needs to be inverted, test and change if necessary
    //shooterConfig.inverted(true);
    shooterConfig.smartCurrentLimit(Constants_Shooter.shooterMotorCurrentLimit);
    shooterRoller.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Shooter roller value", Constants_Shooter.shooterLaunchVoltage);
  }

  // A method to set the voltage of the shooter roller
  public void setShooterRoller(double voltage) {
    
    shooterRoller.setVoltage(voltage);
  }

  // A method to stop the rollers
  public Command stop() {
    return Commands.run(() -> {
      shooterRoller.set(0);     
    }, this);
   
  }
  
  // Return a Command that, while scheduled, runs the shooter at the configured speed
  public Command shootFuel() {
    return Commands.run(() -> shooterRoller.set(Constants_Shooter.shooterSpeed), this);
  }

  // Backwards-compatible direct action used by older commands
  public void shootFuelAction() {
    shooterRoller.set(Constants_Shooter.shooterSpeed);
  }

  public void agitateFuel() {
    fuelAgitator.set(Constants_Shooter.fuelAgitatorSpeed);
  }
  public Command reverseAgitator() {
    return Commands.run(() -> {
      fuelAgitator.set(-Constants_Shooter.fuelAgitatorSpeed);
    }, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

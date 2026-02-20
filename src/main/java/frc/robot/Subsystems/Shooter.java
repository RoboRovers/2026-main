// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import frc.robot.Util.Constants.Constants_Shooter;
import frc.robot.Util.RobotMap;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {
  private final SparkMax shooterRoller;
  public final SparkMax fuelAgitator;

  public static final double GRAVITY = 9.8; // Acceleration due to gravity in m/s^2
  public static final double RADIUS = 0.05; // Radius of the launch wheel in meters
  public static final double DELTA_Y = 1.251; // Vertical displacement of the ball in meters
  public static final double THETA = 73; // Tangential angle of release for the curved backing in degrees 
  public static final double MAX_SPEED = 5676; // Maximum motor speed in rpm

  /** Creates a new Shooter Subsystem. */
  
  public Shooter() {
    // create brushed motors for each of the motors on the shooter mechanism
    shooterRoller = new SparkMax(RobotMap.MAP_SHOOTER.shooterSparkMAX, MotorType.kBrushed);
    fuelAgitator = new SparkMax(RobotMap.MAP_SHOOTER.fuelAgitatorSparkMAX, MotorType.kBrushed);
  
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
  
  public Command shootFuel() {
    return Commands.runOnce(() -> {
      shooterRoller.set(Constants_Shooter.shooterSpeed);
  }, this);
}

  public Command agitateFuel() {
    return Commands.runOnce(() -> {
      fuelAgitator.set(Constants_Shooter.fuelAgitatorSpeed);
    }, this);

  }
  public Command reverseAgitator() {
    return Commands.runOnce(() -> {
      fuelAgitator.set(-Constants_Shooter.fuelAgitatorSpeed);
    }, this);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static double getMotorRatio(double deltaX) {
      //Calculates the required speed ratio for a given horizontal displacement, assuming:
      //1) negligible air friction, and 2) the ball sticks to the roller such that its exit speed matches the wheel's linear speed
      double squaredRadius = Math.pow(RADIUS, 2);
      double squaredCosine = Math.pow(Math.cos(Math.toRadians(THETA)), 2);
      double denDifference = (deltaX * Math.tan(Math.toRadians(THETA))) - DELTA_Y;
      double num = 0.5 * GRAVITY * Math.pow(deltaX, 2);
      double den = squaredRadius * squaredCosine * denDifference;
      double angularSpeed = (30.0 / Math.PI) * Math.sqrt(num / den); 
      return angularSpeed / MAX_SPEED; //may need to be inverted
  }
}

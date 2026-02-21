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
  private final Limelight LL_Shoot;
  // Runtime adjustable speed for the shooter roller. 
  // Initializedf from constants.
  private double currentShooterSpeed = Constants_Shooter.shooterSpeed;
  /** Creates a new Shooter Subsystem. */
  
  @SuppressWarnings("removal")
  public Shooter() {
    // create brushed motors for each of the motors on the shooter mechanism
    shooterRoller = new SparkMax(RobotMap.MAP_SHOOTER.shooterSparkMAX, MotorType.kBrushless);
    fuelAgitator = new SparkMax(RobotMap.MAP_SHOOTER.fuelAgitatorSparkMAX, MotorType.kBrushless);

    //create the limelight for the shooter
    LL_Shoot = new Limelight(Constants_Shooter.CAMERA_NAME);
  
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
   
    // Return a Command that, while scheduled, runs the shooter at the speed calculated from the horizontal displacement from the hub.
    // The getMotorRatio() dynamically adjusts speed based on the robot's position using projectile physics.
    public Command shootFuel() {
      /** return Commands.run(() -> shooterRoller.set(Shooter.getMotorRatio(LL_Shoot.getDeltaX(Constants_Shooter.TAG_HEIGHT, 
        Constants_Shooter.CAMERA_HEIGHT, Constants_Shooter.CAMERA_ANGLE))), this); **/
      return Commands.run(() -> shooterRoller.set(Shooter.getMotorRatio(0.5)), this);
     }

  // Backwards-compatible direct action used by older commands
  public void shootFuelAction() {
    shooterRoller.set(currentShooterSpeed);
  }

  /** Adjust the shooter speed by a delta (e.g. +0.01 or -0.01). Clamped to [-1.0, 1.0]. */
  public void adjustSpeed(double delta) {
    currentShooterSpeed += delta;
    if (currentShooterSpeed > 1.0) currentShooterSpeed = 1.0;
    if (currentShooterSpeed < -1.0) currentShooterSpeed = -1.0;
    SmartDashboard.putNumber("Shooter roller value", currentShooterSpeed);
  }

  public double getCurrentShooterSpeed() {
    return currentShooterSpeed;
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
      double squaredRadius = Math.pow(Constants_Shooter.RADIUS, 2);
      double squaredCosine = Math.pow(Math.cos(Math.toRadians(Constants_Shooter.THETA)), 2);
      double denDifference = (deltaX * Math.tan(Math.toRadians(Constants_Shooter.THETA))) - (Constants_Shooter.DELTA_Y + 0.2); //the 0.2 ensures that the ball always follows a feasible path into the hub and accounts for AF
      double num = 0.5 * Constants_Shooter.GRAVITY * Math.pow(deltaX, 2);
      double den = squaredRadius * squaredCosine * denDifference;
      double angularSpeed = (30.0 / Math.PI) * Math.sqrt(num / den);

      //Keep the speed within bounds
      if (angularSpeed > Constants_Shooter.MAX_SPEED) {
        angularSpeed = Constants_Shooter.MAX_SPEED;
      } 
      return angularSpeed / Constants_Shooter.MAX_SPEED; //entire block may need to be inverted
  }
}

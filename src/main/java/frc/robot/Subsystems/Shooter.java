// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import frc.robot.Util.Constants.Constants_Shooter;
import frc.robot.Util.RobotMap;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {
  private final SparkFlex shooterOut;
  public final SparkFlex fuelAgitator;
  private final SparkMax shooterIntake;
  private final Limelight LL_Shoot;
  public boolean reverseToggle;
  private SparkClosedLoopController shooterOutPidController;
  private double currentShooterSpeed = Constants_Shooter.shooterSpeed;

  
  public Shooter() {
    // create brushless motors for each of the motors on the shooter mechanism
    shooterOut = new SparkFlex(RobotMap.MAP_SHOOTER.shooterOutSparkFLEX, MotorType.kBrushless);
    shooterOutPidController = shooterOut.getClosedLoopController();
    fuelAgitator = new SparkFlex(RobotMap.MAP_SHOOTER.fuelAgitatorSparkFLEX, MotorType.kBrushless);
    shooterIntake = new SparkMax(RobotMap.MAP_SHOOTER.shooterIntakeSparkMAX, MotorType.kBrushless);
    //create the limelight for the shooter
    LL_Shoot = new Limelight(Constants_Shooter.CAMERA_NAME);

    SparkMaxConfig shooterintakeConfig = new SparkMaxConfig();
    shooterintakeConfig.idleMode(IdleMode.kCoast);
    shooterintakeConfig.inverted(false);
    shooterintakeConfig.smartCurrentLimit(Constants_Shooter.shooterMotorCurrentLimit);
    shooterIntake.configure(shooterintakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig shooterOutConfig = new SparkFlexConfig();
    shooterOutConfig.idleMode(IdleMode.kCoast);
    shooterOutConfig.inverted(false);
    shooterOutConfig.smartCurrentLimit(Constants_Shooter.shooterMotorCurrentLimit);
    shooterOutConfig.closedLoop.p(Constants_Shooter.kP);
    shooterOutConfig.closedLoop.i(Constants_Shooter.kI);
    shooterOutConfig.closedLoop.d(Constants_Shooter.kD);
    shooterOutConfig.closedLoop.velocityFF(Constants_Shooter.kFF);
    shooterOutConfig.closedLoop.outputRange(-1, 1);
    shooterOut.configure(shooterOutConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Shooter roller value", Constants_Shooter.shooterLaunchVoltage);
  }

  // un used
  // A method to set the voltage of the shooter roller
  public void setShooterRoller(double voltage) {
    shooterOut.setVoltage(voltage);
  }

  // A method to stop the rollers
  public void stop() {
    shooterOut.set(0);
    shooterIntake.set(0);
  }
   
    // Return a Command that, while scheduled, runs the shooter at the speed calculated from the horizontal displacement from the hub.
    public Command calculatedShootFuel() {
        currentShooterSpeed = Shooter.getMotorRatio(LL_Shoot.getDeltaX(Constants_Shooter.TAG_HEIGHT, 
          Constants_Shooter.CAMERA_HEIGHT, Constants_Shooter.CAMERA_ANGLE));
        return Commands.run(() -> shooterOut.set(currentShooterSpeed), this);
     }
     
     public void remoteShootFuel() {
        // TOOD: To switch from manual voltage control (which sags as your battery drains) to PID 
        // velocity control we need to replace the .set() method with .setReference(). However, 
        // .setReference() requires a velocity in RPM which we need to figure out. 
        shooterOut.set(currentShooterSpeed);
        shooterIntake.set(Constants_Shooter.shooterIntakeSpeed);
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
  
  public void setCurrentShooterSpeed(double speed) {
    currentShooterSpeed = speed;
  }

  public boolean getReverse() {
    return reverseToggle;
  }

  public Command reverseAgitator() {
    return Commands.startEnd(() -> {
      if (reverseToggle)
      {
        fuelAgitator.set(0);
        reverseToggle= false;
      } else
      {
        fuelAgitator.set(Constants_Shooter.fuelAgitatorReversedSpeed);
        reverseToggle = true;
        
      }

      },
      () -> {
        fuelAgitator.set(0);
        
      }, this);
  }

  public Command manualReverseAgitator() {
    return Commands.startEnd(() -> {
      fuelAgitator.set(Constants_Shooter.manualFuelAgitatorReverseSpeed);
    },
    () -> {
      fuelAgitator.set(0);
    }, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", currentShooterSpeed);
  }
  
  public static double getMotorRatio(double xDist) {
      //Calculates the required speed ratio for a given horizontal displacement, assuming:
      //1) negligible air friction, and 2) the ball sticks to the roller such that its exit speed matches the wheel's linear speed
      double squaredRadius = Math.pow(Constants_Shooter.RADIUS, 2);
      double squaredCosine = Math.pow(Math.cos(Math.toRadians(Constants_Shooter.THETA)), 2);
      double denDifference = (xDist * Math.tan(Math.toRadians(Constants_Shooter.THETA))) - (Constants_Shooter.DELTA_Y + 0.2); //the 0.2 ensures that the ball always follows a feasible path into the hub and accounts for AF
      double num = 0.5 * Constants_Shooter.GRAVITY * Math.pow(xDist, 2);
      double den = squaredRadius * squaredCosine * denDifference;
      double angularSpeed = (30.0 / Math.PI) * Math.sqrt(num / den);

      //Keep the speed within bounds
      if (angularSpeed > Constants_Shooter.MAX_SPEED) {angularSpeed = Constants_Shooter.MAX_SPEED;}
      return angularSpeed / Constants_Shooter.MAX_SPEED; //entire block may need to be inverted
  }
  
  
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.HashMap;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj2.command.Command;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    
  public static final class Constants_Module {
    public static final double wheelRadiusMeters = 0.05000625; //Inches; 1 31/32; 1.96875 1.96875 to meters = 0.05000625
    public static final double wheelCircumferenceMeters = 2*Math.PI*wheelRadiusMeters;
    public static final double driveGearRatio = 4.59; //4.59 for Swerve X, 6.75 for sds
    public static final double DRIVE_ROT_2_METER = (wheelCircumferenceMeters);
    public static final double DRIVE_MPS_2_RPS = driveGearRatio/wheelCircumferenceMeters;

    public static final double STEER_GEAR_RATIO = 13.3714; //13.3714 for Swerve X, 12.8 for sds
    public static final double STEER_TO_DEGREES = 360 / STEER_GEAR_RATIO;
    public static final double STEER__RPM_2_DEG_PER_SEC = STEER_TO_DEGREES / 60;

    //TODO Tune our pid loop for the drives once you add in all the offsets, you can just rotate the wheels to 90 degrees using the flight sticks, then disable and enable the code to set them to 0 degreese and tune off of that vaule
    public static final double P_TURNING = 0.0001;
    public static final double I_TURNING = 0.000001;
    public static final double D_TURNING = 0;
    public static final double FF_TURNING = 0;

    //TODO Dont worry about changing these values
    public static final double S_DRIVE = 0.4;
    public static final double V_DRIVE = 0.124;
    public static final double A_DRIVE = 0.1;
    public static final double P_DRIVE = 0.1;
    public static final double I_DRIVE = 0;
  }


  public static final class Constants_Drive {

    public static final Measure<DistanceUnit> WHEEL_RADIUS = edu.wpi.first.units.Units.Inches.of(1.5);
    public static final double COF = 1.2;
    //TODO Measure from the center of each wheel to get these, Front to back for "WHEEL_BASE", Left to right for "TRACK_WIDTH"
    public static final double trackWidth = Units.inchesToMeters(22.9375);  //TODO Update values to what they are for the new robo
      // Distance between left and right wheels
    public static final double wheelBase = Units.inchesToMeters(22.6875);
      // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2, trackWidth / 2), //front left
        new Translation2d(wheelBase / 2, -trackWidth / 2), //front right
        new Translation2d(-wheelBase / 2,  trackWidth / 2), //back left
        new Translation2d(-wheelBase / 2, -trackWidth / 2)); //back right

    public static final double MODULE_RADIUS = Units.inchesToMeters(Constants.Constants_Drive.trackWidth/2); //measured from center of robot to furthest module.

    
    //TODO Test and input all module offsets which range from -1 -> 1, Make sure to read the TODO in the "MODULE" file for more info on zeroing the motors
    public static final double FL_OFFSET = -0.092285; //0.011230;
    public static final double FR_OFFSET = 0.176514; //0.159424;
    public static final double BL_OFFSET = 0.395020; //0.385986;
    public static final double BR_OFFSET = -0.484375; // 0.415527;

    //TODO Invert any motor to match controller output
    public static final boolean FL_STEER_ENCODER_REVERSED = true;//TODO Make sure Counter-Clockwise rotation is considered positive rotation
    public static final boolean FR_STEER_ENCODER_REVERSED = true;
    public static final boolean BL_STEER_ENCODER_REVERSED = true;
    public static final boolean BR_STEER_ENCODER_REVERSED = false;

    public static final boolean FL_DRIVE_ENCODER_REVERSED = false;
    public static final boolean FR_DRIVE_ENCODER_REVERSED = false;
    public static final boolean BL_DRIVE_ENCODER_REVERSED = false;
    public static final boolean BR_DRIVE_ENCODER_REVERSED = false;

    public static final boolean FL_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;//TODO Make sure Counter-Clockwise rotation is considered positive rotation
    public static final boolean FR_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BL_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BR_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;

    public static final double MAX_SPEED_METERS_PER_SEC = 6.949; //6.949 for Swerve X, 4.60248 for sd
    public static final double MAX_ANGULAR_SPEED_RPS = MAX_SPEED_METERS_PER_SEC/trackWidth;

    //For limiting speed while driving
    public static final double TELEDRIVE_MAX_SPEED_METERS_PER_SEC = MAX_SPEED_METERS_PER_SEC / 1.0;
    public static final double TELEDRIVE_MAX_ANGULAR_SPEED_RPS = MAX_ANGULAR_SPEED_RPS / 1.0;
    public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SEC = MAX_SPEED_METERS_PER_SEC/1.50;
    public static final double TELEDRIVE_MAX_ANGULAR_ACCEL_UNITS_PER_SEC = TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SEC/(trackWidth/2);
  }

  public static final class Constants_Climber
  {
    public static final int motorSmartCurrentLimit = 40;  
    public static final double climberFactor = 1.0; //TODO: Get actual value
    public static final double climbSpeed = 0.5; //TODO: Get actual value

    public static final boolean climbInverted = false;
    public static final boolean leftClimbInverted = climbInverted;
    public static final boolean rightClimbInverted = !climbInverted;
  }

  public static final class Constants_Auto 
  {
    public static final double MAX_SPEED_METERS_PER_SEC = Constants_Drive.MAX_SPEED_METERS_PER_SEC;//0.5;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQRD = Constants_Drive.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SEC;//0.25;
    public static final double MAX_ANGULAR_SPEED_RPS =  Constants_Drive.TELEDRIVE_MAX_ANGULAR_SPEED_RPS;
    public static final double MAX_ANGULAR_ACCEL_UNITS_PER_SEC = Constants_Drive.TELEDRIVE_MAX_ANGULAR_ACCEL_UNITS_PER_SEC;

    public static  double P_TRANSLATION = 5.925; //TODO redo PID loop
    public static  double I_TRANSLATION = 0.00;
    public static  double D_TRANSLATION = 0.0;

    public static final double P_THETA = 4.125; //TODO redo PID loop
    public static final double I_THETA = 0.0;
    public static final double D_THETA = 0.0;


    public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = 
            new TrapezoidProfile.Constraints(
                    MAX_ANGULAR_SPEED_RPS,
                    MAX_ANGULAR_ACCEL_UNITS_PER_SEC);
    public static final TrapezoidProfile.Constraints LINEAR_CONSTRAINTS = 
            new TrapezoidProfile.Constraints(
                MAX_SPEED_METERS_PER_SEC,
                MAX_ACCELERATION_METERS_PER_SECOND_SQRD
            );
  }
}

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

    
  public static final class constants_Module {
    public static final double WHEEL_RADIUS_METERS = 0.1016/2; //Inches
    public static final double WHEEL_CIRCUMFRENCE_METERS = 2*Math.PI*WHEEL_RADIUS_METERS;
    public static final double DRIVE_GEAR_RATIO = 4.59; //4.59 for Swerve X, 6.75 for sds
    public static final double DRIVE_ROT_2_METER = (WHEEL_CIRCUMFRENCE_METERS);
    public static final double DRIVE_MPS_2_RPS = DRIVE_GEAR_RATIO/WHEEL_CIRCUMFRENCE_METERS;

    public static final double STEER_GEAR_RATIO = 13.3714; //13.3714 for Swerve X, 12.8 for sds
    public static final double STEER_TO_DEGREES = 360 / STEER_GEAR_RATIO;
    public static final double STEER__RPM_2_DEG_PER_SEC = STEER_TO_DEGREES / 60;

    //TODO Tune our pid loop for the drives once you add in all the offsets, you can just rotate the wheels to 90 degrees using the flight sticks, then disable and enable the code to set them to 0 degreese and tune off of that vaule
    public static final double P_TURNING = 0.0075;
    public static final double I_TURNING = 0.0;
    public static final double D_TURNING = 0.75;
    public static final double FF_TURNING = 0;

    //TODO Dont worry about changing these values
    public static final double S_DRIVE = 0.4;
    public static final double V_DRIVE = 0.124;
    public static final double A_DRIVE = 0.1;
    public static final double P_DRIVE = 0.1;
    public static final double I_DRIVE = 0.1;
  }


  public static final class constants_Drive {

    public static final Measure<DistanceUnit> WHEEL_RADIUS = edu.wpi.first.units.Units.Inches.of(1.5);
    public static final double COF = 1.2;
    //TODO Measure from the center of each wheel to get these, Front to back for "WHEEL_BASE", Left to right for "TRACK_WIDTH"
    public static final double TRACK_WIDTH = Units.inchesToMeters(29);  //TODO Update values to what they are for the new robo
      // Distance between left and right wheels
    public static final double WHEEL_BASE = Units.inchesToMeters(29);
      // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), //front left
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), //front right
        new Translation2d(-WHEEL_BASE / 2,  TRACK_WIDTH / 2), //back left
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); //back right

    public static final double MODULE_RADIUS = Units.inchesToMeters(Constants.constants_Drive.TRACK_WIDTH/2); //measured from center of robot to furthest module.

    
    //TODO Test and input all module offsets which range from -1 -> 1, Make sure to read the TODO in the "MODULE" file for more info on zeroing the motors
    public static final double FL_OFFSET = 0.012207; //0.011230;
    public static final double FR_OFFSET = 0.161377; //0.159424;
    public static final double BL_OFFSET = 0.386230; //0.385986;
    public static final double BR_OFFSET = 0.415283; // 0.415527;

    //TODO Invert any motor to match controller output
    public static final boolean FL_STEER_ENCODER_REVERSED = true;//TODO Make sure Counter-Clockwise rotation is considered positive rotation
    public static final boolean FR_STEER_ENCODER_REVERSED = true;
    public static final boolean BL_STEER_ENCODER_REVERSED = true;
    public static final boolean BR_STEER_ENCODER_REVERSED = true;

    public static final boolean FL_DRIVE_ENCODER_REVERSED = false;
    public static final boolean FR_DRIVE_ENCODER_REVERSED = true;
    public static final boolean BL_DRIVE_ENCODER_REVERSED = false;
    public static final boolean BR_DRIVE_ENCODER_REVERSED = true;

    public static final boolean FL_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;//TODO Make sure Counter-Clockwise rotation is considered positive rotation
    public static final boolean FR_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BL_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BR_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;

    public static final double MAX_SPEED_METERS_PER_SEC = 6.949; //6.949 for Swerve X, 4.60248 for sd
    public static final double MAX_ANGULAR_SPEED_RPS = MAX_SPEED_METERS_PER_SEC/TRACK_WIDTH;

    //For limiting speed while driving
    public static final double TELEDRIVE_MAX_SPEED_METERS_PER_SEC = MAX_SPEED_METERS_PER_SEC / 1.0;
    public static final double TELEDRIVE_MAX_ANGULAR_SPEED_RPS = MAX_ANGULAR_SPEED_RPS / 1.0;
    public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SEC = MAX_SPEED_METERS_PER_SEC/1.50;
    public static final double TELEDRIVE_MAX_ANGULAR_ACCEL_UNITS_PER_SEC = TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SEC/(TRACK_WIDTH/2);
  }
    public static final class constants_intake
    {
      public static final double rollerSpeed = 0.05;

      
      public static final boolean leftIntakeMotorInverted = constants_intake.intakeMotorInverted;
      public static final boolean rightIntakeMotorInverted = constants_intake.intakeMotorInverted;
      public static final boolean rollerIntakeMotorInverted = constants_intake.intakeMotorInverted;
      public static final boolean intakeMotorInverted = false;
      public static final double intakePositionConversionFactor = 1.0; //TODO Set the correct conversion factor for the intake encoder (units -> meters or rotations)
    }
  /*public static final class constants_Elevator
  {
    public static final double ELEVATOR_GEAR_RATIO = 0.2;
    public static final double ELEVATOR_BASE_HEIGHT = 25.0625;
    public static final int ELEVATOR_SPROCKET_TEETH = 22;
    public static final double ELEVATOR_CHAIN_PITCH = 0.25;
    public static final boolean ELEVATOR_CASCADING = true;
    public static final double ELEVATOR_SPROCKET_CIRCUMFRENCE = ELEVATOR_CHAIN_PITCH * ELEVATOR_SPROCKET_TEETH * (ELEVATOR_CASCADING? 2: 1);
    public static final double ELEVATOR_TOLERANCE = 1.5; //inches
    public static final double ROLLER_SENSOR_TOLERANCE = 4; //Inches
    public static final boolean LEFT_INVERTED = false;
    public static final boolean RIGHT_INVERTED = true;
    public static final boolean ROLLER_INVERTED = true;
    // public static final boolean CLEANER_INVERTED = false;

    public static final double ELEVATOR_P = 0.04; //a = -vi^2 / 2 (current-wanted distance)
    public static final double ELEVATOR_I = 0.0;
    public static final double ELEVATOR_D = 0;
    public static final double ELEVATOR_FF = 0;//Dont worry about this value, keep it 0

    public static final double ROLLER_P = 0.1;
    public static final double ROLLER_I = 0;
    public static final double ROLLER_D = 0;
    public static final double ROLLER_FF = 0;//Dont worry about this value, keep it 0

    // public static final double CLEANER_P = 0;
    // public static final double CLEANER_I = 0;
    // public static final double CLEANER_D = 0;
    // public static final double CLEANER_FF = 0;

    public static final ElevatorPositionGroup SOURCE = new ElevatorPositionGroup(ELEVATOR_BASE_HEIGHT, 0.15);
    public static final ElevatorPositionGroup PREP_NONE = new ElevatorPositionGroup(30, 0);
    public static final ElevatorPositionGroup PREP_L1 = new ElevatorPositionGroup(35.5, -1);
    public static final ElevatorPositionGroup PREP_L2 = new ElevatorPositionGroup(35.5, -1);
    public static final ElevatorPositionGroup PREP_L3 = new ElevatorPositionGroup(51, -1);
    public static final ElevatorPositionGroup CLEAN_L2 = new ElevatorPositionGroup(30, -1);
    public static final ElevatorPositionGroup CLEAN_L3 = new ElevatorPositionGroup(45, -1);
    public static final ElevatorPositionGroup CORAL = new ElevatorPositionGroup(30, 0);
    public static final ElevatorPositionGroup SCORE = new ElevatorPositionGroup(-1, 0.35);
    public static final ElevatorPositionGroup SCORE_L1 = new ElevatorPositionGroup(-1, 0.15);
  }*/


  public static final class constants_Climber 
  {
    public static final double CLIMBER_GEAR_RATIO = 1.0/125.0;
    public static final boolean CLIMBER_INVERTED = false;

    public static final ClimberPositionGroup NONE = new ClimberPositionGroup(0);
    public static final ClimberPositionGroup ACTIVE = new ClimberPositionGroup(-2.25);
  }

  public static final class ClimberPositionGroup
  {
    public double angle;

    public ClimberPositionGroup(double angle)
    {
      this.angle = angle;
    }
  }

  /*public static final class constants_Rollers
  {
    public static final double ROLLER_GEAR_RATIO = 1/3.0;
    public static final double ANGLE_TO_DEGREES = 360.0 * ROLLER_GEAR_RATIO;

    public static final boolean ABS_INVERTED = false; //TODO follow the steps in the elvator subsytem to get this number
    public static final double ABS_OFFSET = 315; //TODO follow the steps in the elvator subsytem to get this number

    public static final boolean ROLLER_INVERTED = false;
    public static final boolean ANGLE_INVERTED = false;

    public static final double ROLLER_ANGLE_TOLERANCE = 1.25;
    public static final double CANRANGE_ROLLERS_DISTANCE_LIMIT = 7; //Inches TODO Change this value to be what we want it to be
    public static final double ROLLER_RPM_TOLERANCE = 5;

    public static final double ROLLER_P = 0;
    public static final double ROLLER_I = 0;
    public static final double ROLLER_D = 0;
    public static final double ROLLER_FF = 0;//Dont worry about this value, keep it 0

  }*/

  public static final class constants_AprilTags{ //[id num, height in inches, coordinate x, coordinate y, heading] inches to meters; use field manual image for id reference
    /*public static final double[];
    public static final double[];
    public static final double[];
    public static final double[];
    public static final double[];
    public static final double[];
    public static final double[];
    public static final double[];
    public static final double[];
    public static final double[];
    public static final double[];*/
  }

  public static final class constants_Limelight{ //TODO check if accurate
    public static final double THETA_P = 4;
    public static final double THETA_I = 0.0002;
    public static final double THETA_D = 0;

    /*// Auto constants
    public static final double X_REEF_ALIGNMENT_P = 0.75;
    public static final double Y_REEF_ALIGNMENT_P = 1.125;
    public static final double Y_REEF_ALIGNMENT_D = 0;
    public static final double ROT_REEF_ALIGNMENT_P = 2.75;

    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
    public static final double X_SETPOINT_REEF_ALIGNMENT = -0.34;  // Vertical pose
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
    public static final double Y_SETPOINT_REEF_ALIGNMENT_RIGHT = 0.3302;  // Horizontal pose
    public static final double Y_SETPOINT_REEF_ALIGNMENT_LEFT = 0.0;  // Horizontal pose
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

    public static final double DONT_SEE_TAG_WAIT_TIME = 1;
    public static final double POSE_VALIDATION_TIME = 0.3;

    public static List<Double> coral_Dimensions = List.of(Math.toRadians(0), 0.3683, -0.1651, 0.22225, 0.3048, 1.4859); //Angle, Dis. Forward, Dis. Right, HeightCamera, HeightReef, HeightSource
    public static List<Double> algae_Dimensions = List.of(Math.toRadians(2),0.15875, 0.06985, 0.1905, 0.4064, 1.419352); //Angle, Dis. Forward, Dis. Right, HeightCamera, HeightAlgae, HeightProcessor

    public static final double leftCoralAngle = 0;
    public static final double rightCoralAngle = 0;

    public static List<Double> getCameraList(String camera)
    {
      return camera.equals("limelight-Coral") ? coral_Dimensions : algae_Dimensions;
    }

    public static double getTargetHeight(String target)
    {
      switch(target)
      {
        case "Reef":
          return coral_Dimensions.get(4);
        case "Source":
          return coral_Dimensions.get(5);
        case "Algae":
          return algae_Dimensions.get(4);
        case "Processor":
          return algae_Dimensions.get(5);
      } 

      return 0;
    }
        */
    public static double getTargetAngle(int targetNum)
    {
      double num = 0;
      /*switch(targetNum)
      {
        case 6 -> num = constants_AprilTags.RED_REEF_6[4];
        case 7 -> num  = constants_AprilTags.RED_REEF_7[4];
        case 8 -> num  = constants_AprilTags.RED_REEF_8[4];
        case 9 -> num  = constants_AprilTags.RED_REEF_9[4];
        case 10 -> num  = constants_AprilTags.RED_REEF_10[4];
        case 11 -> num  = constants_AprilTags.RED_REEF_11[4];

        case 19 -> num = constants_AprilTags.BLUE_REEF_19[4];
        case 18 -> num = constants_AprilTags.BLUE_REEF_18[4];
        case 17 -> num = constants_AprilTags.BLUE_REEF_17[4];
        case 22 -> num = constants_AprilTags.BLUE_REEF_22[4];
        case 21 -> num = constants_AprilTags.BLUE_REEF_21[4];
        case 20 -> num = constants_AprilTags.BLUE_REEF_20[4];
      } */

      return num;
    }
  }


  public static final class constants_Auto {
    public static final double MAX_SPEED_METERS_PER_SEC = constants_Drive.MAX_SPEED_METERS_PER_SEC;//0.5;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQRD = constants_Drive.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SEC;//0.25;
    public static final double MAX_ANGULAR_SPEED_RPS =  constants_Drive.TELEDRIVE_MAX_ANGULAR_SPEED_RPS;
    public static final double MAX_ANGULAR_ACCEL_UNITS_PER_SEC = constants_Drive.TELEDRIVE_MAX_ANGULAR_ACCEL_UNITS_PER_SEC;

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


  /*public static final class constants_StateMachine
  {
    public static final Map<TargetState, RobotState> TARGET_TO_ROBOT_STATE = new HashMap<TargetState, RobotState>();

    static
    {
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_NONE, RobotState.PREP_NONE);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_ALGAE, RobotState.PREP_ALGAE);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_L1, RobotState.PREP_L1);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_L2, RobotState.PREP_L2);
      TARGET_TO_ROBOT_STATE.put(TargetState.PREP_L3, RobotState.PREP_L3);
      // TARGET_TO_ROBOT_STATE.put(TargetState.SOURCE, RobotState.SOURCE);
    //   TARGET_TO_ROBOT_STATE.put(TargetState.
    //   TARGET_TO_ROBOT_STATE.put(TargetState.
    //   TARGET_TO_ROBOT_STATE.put(TargetState.
    //   TARGET_TO_ROBOT_STATE.put(TargetState.
    //   TARGET_TO_ROBOT_STATE.put(TargetState.
    //   TARGET_TO_ROBOT_STATE.put(TargetState.
    }

    public static Map<TargetState, ElevatorPositionGroup> TARGET_TO_PRESET_GROUP = new HashMap<TargetState, ElevatorPositionGroup>();

    static
    {
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_NONE, constants_Elevator.PREP_NONE);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_L1, constants_Elevator.PREP_L1);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_L2, constants_Elevator.PREP_L2);
      TARGET_TO_PRESET_GROUP.put(TargetState.PREP_L3, constants_Elevator.PREP_L3);
      TARGET_TO_PRESET_GROUP.put(TargetState.SOURCE, constants_Elevator.SOURCE);

 
    }
  } */

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int SHOOTER_MOTOR_ID = 5;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int SHOOTER_MOTOR_CURRENT_LIMIT = 60;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double SHOOTER_LAUNCH_VOLTAGE = 10.6;
    public static final double SPIN_UP_SECONDS = 1;
  }
}

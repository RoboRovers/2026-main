package frc.robot.Util;

public class RobotMap
{
    public static final class MAP_CONTROLLER
    {
        public static final int leftJoystick = 0;
        public static final int rightJoystick = 1;
        public static final int xboxController = 2;
    }    

    public static final class MAP_CLIMBER
    {
        public static final int leftClimberMotor = 5;
        public static final int rightClimberMotor = 6;
    }

    public static final class MAP_DRIVETRAIN
    {
        //Front Left - Module 0
        public static final int frontLeftDriveKraken = 1;
        public static final int frontLeftSteerSparkMAX = 1;
        public static final int frontLeftAbsEncoder = 1;
        //Front Right - Module 1
        public static final int frontRightDriveKraken = 2;
        public static final int frontRightSteerSparkMAX = 2;
        public static final int frontRightAbsEncoder = 2;
        //Back Right - Module 3
        public static final int backRightDriveKraken = 3;
        public static final int backRightSteerSparkMAX = 3;
        public static final int backRightAbsEncoder = 3;
        //Back Left - Module 2
        public static final int backLeftDriveKraken = 4;
        public static final int backLeftSteerSparkMAX = 4;
        public static final int backLeftAbsEncoder = 4;
    }
}   

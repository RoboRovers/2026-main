package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Util.RobotMap.MAP_CONTROLLER;

public class Controllers {
    public CommandJoystick leftStick, rightStick;
    public CommandXboxController xbox;
    public Trigger zeroHeading, FO_toggle, resetWheels,
                    climbUp, climbDown, climbStop;

    public Controllers() 
    {
        leftStick = new CommandJoystick(MAP_CONTROLLER.leftJoystick);
        rightStick = new CommandJoystick(MAP_CONTROLLER.rightJoystick);
        xbox = new CommandXboxController(MAP_CONTROLLER.xboxController);
        
        // Initialize the convenience Trigger fields so callers can use them directly
        initialize_Xbox_Controls();
        initialize_left_Joystick_Controls();
    }

    public void initialize_Xbox_Controls()
    {
        climbUp = xbox.rightBumper();
        climbDown = xbox.leftBumper();
        climbStop = xbox.button(7); //Get actual button number
    }   

    public void initialize_left_Joystick_Controls() {
        zeroHeading = leftStick.button(5);
        FO_toggle   = leftStick.button(6);
        resetWheels = leftStick.button(7);
    }
}



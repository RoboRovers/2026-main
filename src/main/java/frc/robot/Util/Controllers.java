package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Util.RobotMap.MAP_CONTROLLER;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class Controllers {
    public CommandJoystick leftStick, rightStick;
    public CommandXboxController xbox;
    public Trigger intakeOut, intakeIn, spinRollers, shootFuel, leftBumper, rightBumper, leftTrigger, rightTrigger;

    public Controllers() 
    {
        leftStick = new CommandJoystick(MAP_CONTROLLER.LEFT_JOYSTICK);
        rightStick = new CommandJoystick(MAP_CONTROLLER.RIGHT_JOYSTICK);
        xbox = new CommandXboxController(MAP_CONTROLLER.XBOX_CONTROLLER);
        // Initialize the convenience Trigger fields so callers can use them directly
        initialize_Xbox_Controls();
        initialize_left_Joystick_Controls();
    }

    public void initialize_Xbox_Controls()
    {
        spinRollers = xbox.leftTrigger();
        shootFuel = xbox.rightTrigger();
        intakeOut = xbox.x();
        intakeIn = xbox.y();
        leftBumper = xbox.leftBumper();
        rightBumper = xbox.rightBumper();
        leftTrigger = xbox.leftTrigger();
        rightTrigger = xbox.rightTrigger();

    }

    public void initialize_left_Joystick_Controls() {
        /*
        leftBumper = leftStick.button(1);
        leftTrigger = leftStick.button(2);
        spinRollers = leftStick.button(3);
        shootFuel = leftStick.button(4);   
        intakeOut = leftStick.button(5);
        intakeIn = leftStick.button(6);
        */
    }
    
}



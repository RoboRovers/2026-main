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

    public Controllers() 
    {
        leftStick = new CommandJoystick(MAP_CONTROLLER.LEFT_JOYSTICK);
        rightStick = new CommandJoystick(MAP_CONTROLLER.RIGHT_JOYSTICK);
        xbox = new CommandXboxController(MAP_CONTROLLER.XBOX_CONTROLLER);

    }

    public void initialize_Xbox_Controls()
    {
        
    }
}



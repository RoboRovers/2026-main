package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.ShooterIncreaseSpeed;
import frc.robot.Subsystems.Shooter;
import frc.robot.Util.RobotMap.MAP_CONTROLLER;

public class Controllers {
    public CommandJoystick leftStick, rightStick;
    public CommandXboxController xbox;
    public Trigger intakeFuel, spinRollers, shootFuel, zeroHeading, FO_toggle, 
    resetWheels, shooterIncreaseSpeed, shooterDecreaseSpeed, manualReverseAgitator, 
    leftBumper, rightBumper, leftTrigger, rightTrigger;

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
        intakeFuel = xbox.leftTrigger();
        shootFuel = xbox.rightTrigger();
        // spinRollers = xbox.leftTrigger();
        // intakeOut = xbox.x();
        // intakeIn = xbox.y();
        manualReverseAgitator = xbox.leftBumper();
        rightBumper = xbox.rightBumper();
        leftTrigger = xbox.leftTrigger();
        rightTrigger = xbox.rightTrigger();
        shooterIncreaseSpeed = xbox.x();
        shooterDecreaseSpeed = xbox.y();
        

    }

    public void initialize_left_Joystick_Controls() {
        zeroHeading = leftStick.button(2);
        FO_toggle   = leftStick.button(3);
        resetWheels = leftStick.button(4);
        /*
        leftBumper = leftStick.button(1);
        leftTrigger = leftStick.button(2);
        spinRollers = leftStick.button(3);
        shootFuel = leftStick.button(4);   
        intakeOut = leftStick.button(5);
        intakeIn = leftStick.button(6);
        */
    }

    public void intitalize_right_Joystick_Controls() {


    }
    
}



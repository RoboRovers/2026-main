package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Util.RobotMap.MAP_CONTROLLER;

public class Controllers {
    public CommandJoystick leftStick, rightStick;
    public CommandXboxController xbox;
    public Trigger intakeFuel, spinRollers, shootFuel, zeroHeading, FO_toggle, 
    resetWheels, shooterIncreaseSpeed, shooterDecreaseSpeed, manualReverseAgitator, 
    toggleAutoAgitator, leftBumper, rightBumper, leftTrigger, rightTrigger,
    intakeOutU, intakeOutUL, intakeOutUR, intakeInD, intakeInDL, intakeInDR,
    reverseSpinRollers, fastSpinRollers;

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
        //intakeFuel = xbox.leftTrigger();

        shootFuel = xbox.rightTrigger(.05);
        //spinRollers = xbox.leftTrigger();
        // intakeOut = xbox.x();
        // intakeIn = xbox.y();

        shootFuel = xbox.leftBumper();
        spinRollers = xbox.leftTrigger();
        reverseSpinRollers = xbox.x();
        fastSpinRollers = xbox.b();

        //manualAgitate = xbox.leftBumper();
        manualReverseAgitator = xbox.rightBumper();
        //leftTrigger = xbox.leftTrigger();
        //rightTrigger = xbox.rightTrigger();

        //toggleAutoAgitator = xbox.x();
        //shooterIncreaseSpeed = xbox.y();
        //shooterDecreaseSpeed = xbox.a();
        

        //toggleAutoAgitator = xbox.x();

        shooterIncreaseSpeed = xbox.y();
        shooterDecreaseSpeed = xbox.a();
    

        intakeOutU = xbox.povUp();
        intakeOutUL = xbox.povUpLeft();
        intakeOutUR = xbox.povUpRight();
    
        intakeInD = xbox.povDown();
        intakeInDL = xbox.povDownLeft();
        intakeInDR = xbox.povDownRight();


    }

    public void initialize_left_Joystick_Controls() {
        zeroHeading = leftStick.button(5);
        FO_toggle   = leftStick.button(6);
        resetWheels = leftStick.button(7);
        //spinRollers = leftStick.button(1);
        // manualReverseAgitator = leftStick.button(1);
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
        toggleAutoAgitator = rightStick.button(2); // should be in sbox for operator controls

    }
    
}



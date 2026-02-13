package frc.robot.Subsystems;

import frc.robot.Util.Constants.constants_intake;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Util.RobotMap;

public class Intake extends SubsystemBase {
    public SparkMax leftIntakeMotor;
    public RelativeEncoder leftIntakeEncoder;
    public SparkClosedLoopController leftIntakePID;
    public SparkMaxConfig leftIntakeConfig;

    public SparkMax rightIntakeMotor;
    public RelativeEncoder rightIntakeEncoder;
    public SparkClosedLoopController rightIntakePID;
    public SparkMaxConfig rightIntakeConfig;

    public SparkMax rollerIntakeMotor;
    public RelativeEncoder rollerIntakeEncoder;
    public SparkClosedLoopController rollerIntakePID;
    public SparkMaxConfig rollerIntakeConfig;
    
    
    public Intake() {
    // Left intake motor setup
    leftIntakeConfig = new SparkMaxConfig();
    leftIntakeConfig.idleMode(IdleMode.kBrake);
    // TODO: set the correct conversion factor for the intake encoder (units -> meters or rotations)
    leftIntakeConfig.encoder.positionConversionFactor(constants_intake.intakePositionConversionFactor);
    leftIntakeConfig.inverted(constants_intake.leftIntakeMotorInverted); //TODO: find out if the motor needs to be inverted
        
    leftIntakeMotor = new SparkMax(RobotMap.MAP_INTAKE.leftIntakeSparkMAX, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    leftIntakeEncoder = leftIntakeMotor.getEncoder();
    leftIntakePID = leftIntakeMotor.getClosedLoopController();
    leftIntakeMotor.configure(leftIntakeConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Right intake motor setup
    rightIntakeConfig = new SparkMaxConfig();
    rightIntakeConfig.idleMode(IdleMode.kBrake);
    // TODO: set the correct conversion factor for the intake encoder (units -> meters or rotations)
    rightIntakeConfig.encoder.positionConversionFactor(constants_intake.intakePositionConversionFactor);
    rightIntakeConfig.inverted(constants_intake.rightIntakeMotorInverted); //TODO: find out if the motor needs to be inverted
        
    rightIntakeMotor = new SparkMax(RobotMap.MAP_INTAKE.rightIntakeSparkMAX, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    rightIntakeEncoder = rightIntakeMotor.getEncoder();
    rightIntakePID = rightIntakeMotor.getClosedLoopController();
    rightIntakeMotor.configure(rightIntakeConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Roller intake motor setup
    rollerIntakeConfig = new SparkMaxConfig();
    rollerIntakeConfig.idleMode(IdleMode.kBrake);
    // TODO: set the correct conversion factor for the intake encoder (units -> meters or rotations)
    rollerIntakeConfig.encoder.positionConversionFactor(constants_intake.intakePositionConversionFactor);
    rollerIntakeConfig.inverted(constants_intake.rollerIntakeMotorInverted);
        
    rollerIntakeMotor = new SparkMax(RobotMap.MAP_INTAKE.rollerIntakeSparkMAX, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    rollerIntakeEncoder = rollerIntakeMotor.getEncoder();
    rollerIntakePID = rollerIntakeMotor.getClosedLoopController();
    rollerIntakeMotor.configure(rollerIntakeConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure intake motor settings here
    }
    public double getPosition()
    {
        return leftIntakeEncoder.getPosition();
    }

    public Command spinRollers()
    {
        return Commands.run(() -> {
        rollerIntakeMotor.set(constants_intake.rollerSpeed);
        }, this);
    }
    public void zeroPosition()
    {
        leftIntakeEncoder.setPosition(2); // TODO: set the correct zero position
        rightIntakeEncoder.setPosition(2); // TODO: set the correct zero position
    }
    
     public Command intakeIn()
    {
        if (getPosition() < constants_intake.retractLimit) { // TODO: set the correct position limit
            return Commands.run(() -> {
                leftIntakeMotor.set(-.05); // between -1 and 1
                rightIntakeMotor.set(-.05);
            }, this); // note `this` makes it require the Intake subsystem
        }
        return Commands.none();
    }

     public Command intakeOut()
    {
        if (getPosition() > constants_intake.extendLimit) { // TODO: set the correct position limit
            return Commands.run(() -> {
                leftIntakeMotor.set(.05);
                rightIntakeMotor.set(.05);
            }, this); // note `this` makes it require the Intake subsystem
        }
        return Commands.none();
    }

    //  public Command stopIntake()
    // {
    //     return Commands.runOnce(()->
    //     {
    //         leftIntakeMotor.set(0);
    //         rightIntakeMotor.set(0);
    //     });
    // }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position", getPosition());
    }

}

package frc.robot.Subsystems;

import frc.robot.Util.Constants.constants_intake;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Util.RobotMap;

public class Intake {
    public SparkMax intakeMotor;
    public RelativeEncoder intakeEncoder;
    public SparkClosedLoopController intakePID;
    public SparkMaxConfig intakeConfig;
    public Intake() {
        intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake);
        // TODO: set the correct conversion factor for the intake encoder (units -> meters or rotations)
        intakeConfig.encoder.positionConversionFactor(1.0);
        intakeConfig.inverted(constants_intake.intakeMotorInverted);
        
        intakeMotor = new SparkMax(RobotMap.map_intake.intakeSparkmax, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
        intakePID = intakeMotor.getClosedLoopController();
        intakeMotor.configure(intakeConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure intake motor settings here
    }


}

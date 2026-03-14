package frc.robot.Subsystems;
import frc.robot.Util.Constants.Constants_Climber;
import frc.robot.Util.RobotMap;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private SparkMax leftClimberMotor;
    private SparkMax rightClimberMotor;
    private RelativeEncoder leftClimberEncoder;

    public Climber()
    {
        //Left Motor Configuration
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.idleMode(IdleMode.kBrake);
        leftConfig.inverted(Constants_Climber.leftClimbInverted);
        leftConfig.smartCurrentLimit(Constants_Climber.motorSmartCurrentLimit);
        leftConfig.encoder.positionConversionFactor(Constants_Climber.climberFactor);

        //Left Motor
        leftClimberMotor = new SparkMax(RobotMap.MAP_CLIMBER.leftClimberMotor, MotorType.kBrushless);
        leftClimberMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftClimberEncoder = leftClimberMotor.getEncoder();

        //Right Motor Configuration
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kBrake);
        rightConfig.inverted(Constants_Climber.rightClimbInverted);
        rightConfig.smartCurrentLimit(Constants_Climber.motorSmartCurrentLimit);
        rightConfig.encoder.positionConversionFactor(Constants_Climber.climberFactor);

        //Right Motor
        rightClimberMotor = new SparkMax(RobotMap.MAP_CLIMBER.rightClimberMotor, MotorType.kBrushless);
        rightClimberMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command climb()
    {
        return Commands.none(); //TODO: Implement climb command
    }

    public Command manualStop()
    {
        return Commands.runOnce(() -> stop());
    }

    public void stop()
    {
        leftClimberMotor.stopMotor();
        rightClimberMotor.stopMotor();
    }

    public void periodic()
    {
        SmartDashboard.putNumber("Climber Position", leftClimberEncoder.getPosition());
    }
}
    

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
    private SparkMax climbMotor;
    private RelativeEncoder climberEncoder;

    public Climber()
    {
        //Motor Configuration
        SparkMaxConfig climbConfig = new SparkMaxConfig();
        climbConfig.idleMode(IdleMode.kBrake);
        climbConfig.inverted(Constants_Climber.climbInverted);
        climbConfig.smartCurrentLimit(Constants_Climber.motorSmartCurrentLimit);
        climbConfig.encoder.positionConversionFactor(Constants_Climber.climberFactor);

        //Motor Instantiation
        climbMotor = new SparkMax(RobotMap.MAP_CLIMBER.climbMotor, MotorType.kBrushless);
        climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climberEncoder = climbMotor.getEncoder();
    }
   
    public void stop()
    {
        climbMotor.stopMotor();
        
    }

    public void climbUp()
    {
        climbMotor.set(Constants_Climber.climbSpeed);
    }

    public void climbDown()
    {
        climbMotor.set(-Constants_Climber.climbSpeed);
    }

    public void periodic()
    {
        SmartDashboard.putNumber("Climber Position", climberEncoder.getPosition());
    }
}
    

package frc.robot.Commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Subsystems.Climber;

public class Climb extends Command {
    private Climber s_Climb;
    private Timer delay = new Timer();
    
    public Climb(Climber s_Climb)
    {
        this.s_Climb = s_Climb;
        addRequirements(s_Climb);
    }
    
    //This should only be made if time is given
    @Override
    public void initialize()
    {
        delay.start();
    }

    @Override
    public void execute()
    {      
       s_Climb.climbUp();
    }

    @Override
    public void end(boolean interrupted)
    {
        s_Climb.stop();
    }   
}

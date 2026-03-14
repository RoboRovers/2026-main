package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Climber;

public class Climb extends Command {
    private Climber theClimb;

    public Climb(Climber theClimb)
    {
        this.theClimb = theClimb;
        addRequirements(theClimb);
    }

    @Override
    public void initialize()
    {
        theClimb.stop();
    }

    @Override
    public void execute()
    {
       Commands.run(() -> theClimb.climb());
    }

    @Override
    public void end(boolean interrupted)
    {
        theClimb.stop();
    }   
}

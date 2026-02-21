package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;
import frc.robot.Util.Constants.Constants_Shooter;


public class Shoot extends Command {
public final Shooter shooter;
private Timer delay = new Timer();


public Shoot(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
    
}

    @Override
    public void initialize() {
        delay.start();
        
    }

    @Override
    public void execute() {
        // call the direct action so this command continues to use the motor directly
        shooter.shootFuel();
        if (delay.get() > 2)
        {
            shooter.fuelAgitator.set(Constants_Shooter.fuelAgitatorSpeed);
        }
    
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.fuelAgitator.set(Constants_Shooter.fuelAgitatorReversedSpeed);
    }

    @Override
    public boolean isFinished()
    {
        return false;
}
}

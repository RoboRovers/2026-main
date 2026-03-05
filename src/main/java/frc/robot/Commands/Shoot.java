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
        if (delay.get()>1)
        {
            delay.restart();
        }else {
            delay.start();
        }

        
        shooter.fuelAgitator.set(0);
        

        //shooter.magicCarpet.set(0);    

    }

    @Override
    public void execute() {
        // call the direct action so this command continues to use the motor directly
        shooter.remoteShootFuel();
        if (delay.get() > 1) // delay is in seconds
        {
            shooter.fuelAgitator.set(Constants_Shooter.fuelAgitatorSpeed);
            //shooter.magicCarpet.set(Constants_Shooter.fuelAgitatorSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        
        if (shooter.getReverse())
        {
            shooter.fuelAgitator.set(Constants_Shooter.fuelAgitatorReversedSpeed);
           // shooter.magicCarpet.set(Constants_Shooter.fuelAgitatorReversedSpeed);
        }
        else
        {
            shooter.fuelAgitator.set(0);
            //shooter.magicCarpet.set(0);
        }
    }
    @Override
    public boolean isFinished()
    {
        return false;
}
}

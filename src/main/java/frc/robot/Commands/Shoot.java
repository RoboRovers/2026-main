package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class Shoot extends Command {
public final Shooter shooter;


public Shoot(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
}

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooter.shootFuel();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.fuelAgitator.set(0);
    }

    @Override
    public boolean isFinished()
    {
        return false;
}
}

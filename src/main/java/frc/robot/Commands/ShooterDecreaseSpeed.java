package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;
import frc.robot.Util.Constants;

/** Decrease the shooter speed by -0.01 when triggered. */
public class ShooterDecreaseSpeed extends Command {
  private final Shooter shooter;
  private final double delta = -Constants.Constants_Shooter.motorDelta;

  public ShooterDecreaseSpeed(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.adjustSpeed(delta);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    // run once
    return true;
  }
}

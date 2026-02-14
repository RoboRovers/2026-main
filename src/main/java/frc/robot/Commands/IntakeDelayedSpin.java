package frc.robot.Commands;

// Using the intake encoder position instead of a Timer
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Util.Constants.Constants_Intake;

/**
 * Command that, while held, runs the intake out motors immediately, and after a delay
 * starts the intake rollers. The command ends when interrupted (button released).
 */
public class IntakeDelayedSpin extends Command {
  private final Intake intake;
  private boolean rollersStarted = false;
  private final double positionThreshold = 50.0;

  public IntakeDelayedSpin(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    rollersStarted = false;
    // start the intake out motors immediately
    intake.leftIntakeMotor.set(.05);
    intake.rightIntakeMotor.set(.05);
  }

  @Override
  public void execute() {
    // keep intake out running while held
    intake.leftIntakeMotor.set(.05);
    intake.rightIntakeMotor.set(.05);

    if (!rollersStarted && intake.getPosition() > positionThreshold) {
      rollersStarted = true;
      // start rollers
      intake.rollerIntakeMotor.set(Constants_Intake.rollerSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // stop all intake motors on release/interruption
    intake.leftIntakeMotor.set(0);
    intake.rightIntakeMotor.set(0);
    intake.rollerIntakeMotor.set(0);
    // nothing additional to stop; encoder doesn't need stopping
  }

  @Override
  public boolean isFinished() {
    // This command is intended to run until the trigger releases (interrupts it).
    return false;
  }
}

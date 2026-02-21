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
    // Start extending the intake immediately on trigger press.
    // Check the intake position, don't try to extend if we're 
    // already at or beyond the threshold position.
    if (intake.getPosition() < positionThreshold) {
        intake.leftIntakeMotor.set(.05);
        intake.rightIntakeMotor.set(.05);
    }
  }

  @Override
  public void execute() {
    // While rollers haven't started yet, extend the intake until position threshold
    if (!rollersStarted) {
      if (intake.getPosition() < positionThreshold) {
        // keep extending
        intake.leftIntakeMotor.set(.05);
        intake.rightIntakeMotor.set(.05);
      } else {
        // reached threshold: stop intake motors and start rollers
        intake.leftIntakeMotor.set(0);
        intake.rightIntakeMotor.set(0);
        intake.rollerIntakeMotor.set(Constants_Intake.rollerSpeed);
        rollersStarted = true;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    // stop the roller spinner immediately on release/interruption
    intake.rollerIntakeMotor.set(0);
    // stop the intake extension motors before starting retract
    intake.leftIntakeMotor.set(0);
    intake.rightIntakeMotor.set(0);
    // Do not schedule the retract here (schedule() is deprecated). The
    // trigger binding in RobotContainer will start `IntakeReturn` on release.
  }

  @Override
  public boolean isFinished() {
    // This command is intended to run until the trigger releases (interrupts it).
    return false;
  }
}

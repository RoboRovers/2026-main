package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Util.Constants.Constants_Intake;

/**
 * Command that runs the intake motors in reverse until the intake position
 * reaches the given retracted position (default 2), then stops the motors.
 */
public class IntakeReturn extends Command {
  private final Intake intake;
  private final double targetPosition = Constants_Intake.retractLimit;
  private final double retractSpeed = Constants_Intake.intakeRetractSpeed;

  public IntakeReturn(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    // Start retracting immediately when button is released.
    // Check the intake position, don't try to retract if 
    // we're already at the target position.
    if (intake.getPosition() > targetPosition) {
      intake.leftIntakeMotor.set(retractSpeed);
      intake.rightIntakeMotor.set(retractSpeed);
    }
  }

  @Override
  public void execute() {
    // Ensure motors keep retracting until we reach the target 
    // position, then stop them.
    if (intake.getPosition() > targetPosition) {
      intake.leftIntakeMotor.set(retractSpeed);
      intake.rightIntakeMotor.set(retractSpeed);
    } else {
      intake.leftIntakeMotor.set(0);
      intake.rightIntakeMotor.set(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // stop intake motors when finished or interrupted
    intake.leftIntakeMotor.set(0);
    intake.rightIntakeMotor.set(0);
  }

  @Override
  public boolean isFinished() {
    return intake.getPosition() <= targetPosition;
  }
}

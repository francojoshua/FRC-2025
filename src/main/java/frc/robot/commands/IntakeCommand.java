

package frc.robot.commands;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class IntakeCommand extends Command {
  
  private final IntakeSubsystem intakeSubsystem_instance;


  public IntakeCommand(IntakeSubsystem intakeSubsystem) {
      intakeSubsystem_instance = intakeSubsystem;
      addRequirements(intakeSubsystem_instance);
  }




  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      intakeSubsystem_instance.set_speed();
  }




  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem_instance.stop();
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

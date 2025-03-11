package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {
    private final ArmSubsystem armSubsystem_instance;

  public ArmCommand(ArmSubsystem armSubsystem) {
    armSubsystem_instance = armSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      armSubsystem_instance.set_speed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      armSubsystem_instance.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

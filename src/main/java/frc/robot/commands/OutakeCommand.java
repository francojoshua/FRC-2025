// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class OutakeCommand extends Command {
   
  private final IntakeSubsystem intakeSubsystem_instance;
 

  public OutakeCommand(IntakeSubsystem intakeSubsystem) {
    intakeSubsystem_instance = intakeSubsystem;
    addRequirements(intakeSubsystem_instance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem_instance.set_outake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

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

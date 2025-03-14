// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveTeleOpCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	
	private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	private final ArmSubsystem armSubsystem = new ArmSubsystem();
	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	//private final ElevatorSubsystem elevatorSubsytem = new ElevatorSubsystem();
	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController controller =
			new CommandXboxController(OperatorConstants.kDriverControllerPort);

	private final Joystick joystick = new Joystick(OperatorConstants.kDriverControllerPort);
	
	private final SendableChooser<Command> chooser = new SendableChooser<>();

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the trigger bindings
	
		
	
		swerveSubsystem.setDefaultCommand(
				new SwerveTeleOpCommand(swerveSubsystem, () -> controller.getLeftY(),
						() -> controller.getLeftX(), () -> controller.getRightX(), () -> joystick.getRawButton(ControllerConstants.LY_ID))
		);

		//elevatorSubsytem.setDefaultCommand(new ElevatorCommand(elevatorSubsytem));

		
		configureBindings();
		configureAuto();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
	 */
	private void configureBindings() {
		// Schedule `ExampleCommand` when `exampleCondition` changes to `true`
		// new Trigger(m_exampleSubsystem::exampleCondition)
		// .onTrue(new ExampleCommand(m_exampleSubsystem));
		

		controller.x().onTrue(Commands.runOnce(swerveSubsystem::zeroHeading).andThen(Commands.runOnce(swerveSubsystem::useGyroHeading)));
		controller.rightBumper().onTrue(Commands.runOnce(swerveSubsystem::toggleSlowMode));
		controller.leftTrigger().whileTrue(new IntakeCommand(intakeSubsystem));
		controller.rightTrigger().whileTrue(new ArmCommand(armSubsystem));
		//controller.a().onTrue(new InstantCommand( ()-> elevatorSubsytem.change_setpoint(Elevator.level_1)));
		//controller.b().onTrue(new InstantCommand( ()-> elevatorSubsytem.change_setpoint(Elevator.level_2)));
		//controller.y().onTrue(new InstantCommand( ()-> elevatorSubsytem.change_setpoint(Elevator.level_3)));
		//controller.leftBumper().onTrue(new InstantCommand( ()-> elevatorSubsytem.change_setpoint(Elevator.height_minmum)));
		//controller.povDown().onTrue(Commands.runOnce(elevatorSubsytem::decrement_setpoint));
		//controller.povUp().onTrue(Commands.runOnce(elevatorSubsytem::increment_setpoint));


		// Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
		// cancelling on release.
		// m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
	}

	private void configureAuto() {

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return chooser.getSelected();
	}
}

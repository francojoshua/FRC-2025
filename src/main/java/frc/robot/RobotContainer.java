// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;





import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveTeleOpCommand;
import frc.robot.subsystems.SwerveSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
	// The robot's subsystems and commands are defined here...
	private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
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
						() -> controller.getLeftX(), () -> controller.getRightX(), () -> joystick.getRawButton(ControllerConstants.LY_ID)));

		
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
		

		controller.a().onTrue(Commands.runOnce(swerveSubsystem::zeroHeading).andThen(Commands.runOnce(swerveSubsystem::useGyroHeading)));
		controller.rightBumper().onTrue(Commands.runOnce(swerveSubsystem::toggleSlowMode));



		// Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
		// cancelling on release.
		// m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
	}
	public Command createMoveForwardAuto() {
		return new SequentialCommandGroup(
			new RunCommand(() -> swerveSubsystem.driveRobotRelative(
				new ChassisSpeeds(0.0,-1.0, 0.0)), swerveSubsystem)
				.withTimeout(0.75), // Move forward for 2 seconds
			new InstantCommand(swerveSubsystem::stopModules, swerveSubsystem) // Stop the robot
			);
	}
	
	private void configureAuto() {
		chooser.setDefaultOption("Move Forward Auto", createMoveForwardAuto());
		SmartDashboard.putData("Auto Mode", chooser);
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

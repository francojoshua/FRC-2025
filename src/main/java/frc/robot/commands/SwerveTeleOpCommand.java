
package frc.robot.commands;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveTeleOpCommand extends Command {

	private final SwerveSubsystem swerveSubsystem;
	private final Supplier<Double> leftAxisX, leftAxisY, rightAxisX;

	private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;
	private final Supplier<Boolean> slowModeSupplier;

	public SwerveTeleOpCommand(SwerveSubsystem swerveSubsystem, Supplier<Double> leftAxisX,
			Supplier<Double> leftAxisY, Supplier<Double> rightAxisX, Supplier<Boolean> slowModeSupplier) {
		this.swerveSubsystem = swerveSubsystem;
		this.leftAxisX = leftAxisX;
		this.leftAxisY = leftAxisY;
		this.rightAxisX = rightAxisX;

		this.xLimiter = new SlewRateLimiter(2);
		this.yLimiter = new SlewRateLimiter(3);
		this.rotLimiter = new SlewRateLimiter(2);

		this.slowModeSupplier = slowModeSupplier;

		addRequirements(swerveSubsystem);
	}

	@Override
	public void initialize() {
		//swerveSubsystem.zeroHeading();
	}

	@Override
	public void execute() {

		// X FORWARD, Y SIDEAWAYS
		double vxSpeed = -leftAxisX.get();
		double vySpeed = -leftAxisY.get();
		double rot = -rightAxisX.get();

		// Deadband
		vxSpeed = Math.abs(vxSpeed) > ControllerConstants.kDeadband ? vxSpeed : 0.0;
		vySpeed = Math.abs(vySpeed) > ControllerConstants.kDeadband ? vySpeed : 0.0;
		rot = Math.abs(rot) > ControllerConstants.kDeadband ? rot : 0.0;

		// use slew limiter here
		vxSpeed = xLimiter.calculate(vxSpeed) * (DriveConstants.kMaxSpeedMetersPerSecond);
		vySpeed = yLimiter.calculate(vySpeed) * (DriveConstants.kMaxSpeedMetersPerSecond);
		rot = rotLimiter.calculate(rot)
				* (DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond);

		if (slowModeSupplier.get()) {
			vxSpeed *= 0.10;
			vySpeed *= 0.10;
			rot *= 0.10;
		}

		if (vxSpeed == 0.0 & vySpeed == 0.0 & rot == 0.0) {
            swerveSubsystem.setModuleStates(
                    new SwerveModuleState(0.001000001, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0.001000001, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0.001000001, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0.001000001, Rotation2d.fromDegrees(45)));
			return;
		}

		ChassisSpeeds chassisSpeeds;
		if (DriveConstants.kIsFieldCentric) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxSpeed, vySpeed, rot,
				swerveSubsystem.getOdometer2d());
		}
		else {
			chassisSpeeds = new ChassisSpeeds(vxSpeed, vySpeed, rot);
		}
		

		SwerveModuleState[] moduleStates =
				DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

		swerveSubsystem.setModuleStates(moduleStates);
	}

	@Override
	public void end(boolean interrupted) {
		swerveSubsystem.stopModules();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

}

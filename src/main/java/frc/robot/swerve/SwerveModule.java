package frc.robot.swerve;

import static edu.wpi.first.units.Units.Radian;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {

	private final SparkMax driveMotor;
	private final SparkMax angleMotor;

	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder angleEncoder;

	// private final PIDController anglePIDController;

	private final SparkClosedLoopController anglePIDController;
	private final SparkClosedLoopController drivePIDController;

	private final CANcoder absoluteEncoder;

	private final boolean inverseAbsoluteEncoder;
	private final double absoluteEncoderOffset;

	public SwerveModule(int driveMotorId, int angleMotorId, int absoluteEncoderId,
			double absoluteEncoderOffset, boolean inverseDriveMotor, boolean inverseAngleMotor,
			boolean inverseAbsoluteEncoder) {
		driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
		angleMotor = new SparkMax(angleMotorId, MotorType.kBrushless);

		driveEncoder = driveMotor.getEncoder();
		angleEncoder = angleMotor.getEncoder();

		anglePIDController = angleMotor.getClosedLoopController();
		drivePIDController = driveMotor.getClosedLoopController();


		absoluteEncoder = new CANcoder(absoluteEncoderId);

		this.inverseAbsoluteEncoder = inverseAbsoluteEncoder;
		this.absoluteEncoderOffset = absoluteEncoderOffset;

		SparkMaxConfig angleConfig = new SparkMaxConfig();
		
		angleConfig
			.inverted(inverseAngleMotor)
			.idleMode(IdleMode.kCoast)
			.smartCurrentLimit(20);
		angleConfig.encoder
			.positionConversionFactor(SwerveConstants.kAnglePositionConversionFactor)
			.velocityConversionFactor(SwerveConstants.kAngleVelocityConversionFactor);
		angleConfig.closedLoop
			.pidf(SwerveConstants.kPAngle, 0, 0, 0)
			.positionWrappingEnabled(true)
			.positionWrappingMaxInput(2 * Math.PI)
			.positionWrappingMinInput(0.0)
			.outputRange(-1, 1)
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
	
		angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		SparkMaxConfig driveConfig = new SparkMaxConfig();

		driveConfig
			.inverted(inverseDriveMotor)
			.idleMode(IdleMode.kBrake)
			.smartCurrentLimit(50);
		driveConfig.encoder
			.positionConversionFactor(SwerveConstants.kDrivePositionConversionFactor)
			.velocityConversionFactor(SwerveConstants.kDriveVelocityConversionFactor);
		driveConfig.closedLoop
			.pidf(0.3, 0.0, 0.0, 1 / 5676)
			.outputRange(-1, 1)
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
		
		driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		angleEncoder.setPosition(getAbsolutePosition());

		resetEncoders();
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAnglePosition()));
	}

	public void setDesiredState(SwerveModuleState state, boolean isSlowModeEnabled) {
		if (Math.abs(state.speedMetersPerSecond) < 0.001) {
			stop();
			return;
		}

		// Optimizes the angle of the motor. If it can turn -45 instead of 135 degrees.
		// It will.
		state = SwerveModuleState.optimize(state, new Rotation2d(getAnglePosition()));

		//driveMotor.set(state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond);
		//angleMotor.set(anglePIDController.calculate(getAnglePosition(), state.angle.getRadians()));

		drivePIDController.setReference(state.speedMetersPerSecond * (isSlowModeEnabled ? 0.3 : 1), ControlType.kVelocity);
		anglePIDController.setReference(state.angle.getRadians(), ControlType.kPosition);

		SmartDashboard.putString("Swerve " + absoluteEncoder.getDeviceID() + " state",
				state.toString());
		
		SmartDashboard.putNumber("Absolute Position " + absoluteEncoder.getDeviceID(), Units.radiansToDegrees(getAbsolutePosition()));
	}

	public double getDrivePosition() {
		return driveEncoder.getPosition();
	}

	public double getDriveVelocity() {
		return driveEncoder.getVelocity();
	}

	public double getAnglePosition() {
		return angleEncoder.getPosition();
	}

	public double getAngleVelocity() {
		return angleEncoder.getVelocity();
	}

	public double getAbsolutePosition() {
		StatusSignal<Angle> angleSupplier = absoluteEncoder.getAbsolutePosition();

		double angle = angleSupplier.getValue().in(Radian);

		// if (Robot.isReal()) {
		// 	angle = angle - absoluteEncoderOffset;
		// }

		angle = angle * (inverseAbsoluteEncoder ? -1.0 : 1.0);

		return angle;
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAnglePosition()));
	}

	public SparkClosedLoopController getAnglePIDController() {
		return anglePIDController;
	}

	public void resetEncoders() {
		driveEncoder.setPosition(0.0);
		angleEncoder.setPosition(getAbsolutePosition());
	}

	public void stop() {
		driveMotor.set(0.0);
		angleMotor.set(0.0);
	}
}

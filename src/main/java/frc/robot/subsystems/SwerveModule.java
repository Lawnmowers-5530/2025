// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Swerve;

/**
 * Holds methods to easily change the state of modules.
 */
public class SwerveModule extends SubsystemBase {

	static final class SwerveModuleConstants extends Swerve.SwerveModule {
	};

	static final class SwerveAnglePIDConstants extends Swerve.SwerveModule.SwerveAnglePIDConstants {
	};

	private final PIDController anglePID = new PIDController(
			SwerveAnglePIDConstants.p,
			SwerveAnglePIDConstants.i,
			SwerveAnglePIDConstants.d);

	private TalonFX drive;
	private SparkMax rotate;
	private CANcoder canCoder;
	private double angleOffset;
	private boolean slowMode;

	public SwerveModule(int driveMotorID, int turnMotorID, int canCoderID, double angleOffset, boolean inverted) { // initialize
																													// module
		drive = new TalonFX(driveMotorID);
		rotate = new SparkMax(turnMotorID, MotorType.kBrushless);

		//var talonFXConfigurator = drive.getConfigurator();
		//var motorConfigs = new MotorOutputConfigs();
		//motorConfigs.Inverted = (inverted ? InvertedValue.Clockwise_Positive
		//		: InvertedValue.CounterClockwise_Positive);
		//talonFXConfigurator.apply(motorConfigs);

		this.canCoder = new CANcoder(canCoderID);

		anglePID.enableContinuousInput(0, Math.PI * 2);
		this.angleOffset = angleOffset;

		TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
		MotorOutputConfigs talonOutputConfig = new MotorOutputConfigs();
		talonOutputConfig.NeutralMode = NeutralModeValue.Brake;
		talonOutputConfig.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive; // invert the motor output because theres
																				// an extra gear on these swerve modules
		driveConfiguration.withMotorOutput(talonOutputConfig);
		drive.getConfigurator().apply(driveConfiguration);

		SparkMaxConfig rotateConfig = new SparkMaxConfig();
		rotateConfig.idleMode(IdleMode.kBrake); // same invert logic as the talons
		rotateConfig.inverted(true);
		rotate.configure(rotateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

	}

	/**
	 * Directly sets speed of drive motor based on input, and sets wheel angle with
	 * PID controller
	 * 
	 * @param state {@link SwerveModuleState} object with speed and angle
	 */
	public void setState(SwerveModuleState state) {
		state.optimize(getTurningPosition()); // swerve module optimization prevents the
												// wheel
												// from ever turning more than 180 degrees,
												// so it
												// always chooses the shortest angle to
												// rotate to
		double clamped_out = Math.max(-1.0, Math.min(1.0, state.speedMetersPerSecond));

		SmartDashboard.putNumber("smps", state.speedMetersPerSecond);

		drive.set(slowMode ? clamped_out * SwerveModuleConstants.slowModeScaleFactor : clamped_out); // set speed of drive motor
		// System.out.println(state.speedMetersPerSecond);
		double pidOut = anglePID.calculate(getTurningPosition().getRadians(), state.angle.getRadians());
		rotate.set(pidOut);
	}

	/**
	 * Get absolute turning position from {@link CANcoder}
	 * 
	 * @return Absolute angle of module including angleOffset
	 */
	public Rotation2d getTurningPosition() {
		return new Rotation2d(
				((this.canCoder.getAbsolutePosition().getValueAsDouble()) * Math.PI * 2 + this.angleOffset));
	}

	/**
	 * Get velocity of drive motor in m/s on ground
	 * 
	 * @return Velocity of drive motor
	 */
	public double getVelocity() { // convert rpm to m/s
		return drive.getVelocity().getValueAsDouble() * SwerveModuleConstants.conversionFactor;
	}

	/**
	 * 
	 * @return The angle offset value of the module
	 */
	public double getOffset() {
		return this.angleOffset;
	}

	/**
	 * Total non-absolute distance travelled by drive motor. Only used for creating
	 * a {@link SwerveModulePosition} object for PathPlanner
	 * 
	 * @return Total non-absolute distance travelled by drive motor
	 */
	public double getDistance() {
		return drive.getPosition().getValueAsDouble() * SwerveModuleConstants.conversionFactor;
	}

	/**
	 * 
	 * @return {@link SwerveModulePosition} object containing distance travelled by
	 *         drive motor and current wheel angle
	 */
	public SwerveModulePosition getPos() {
		return new SwerveModulePosition(getDistance(), getTurningPosition());
	}

	/**
	 * 
	 * @return {@link SwerveModuleState} object containing drive motor velocity and
	 *         current wheel angle
	 */
	public SwerveModuleState getState() {
		return new SwerveModuleState(getVelocity(), getTurningPosition());
	}

	public void setSlowMode(boolean slowMode) {
		this.slowMode = slowMode;
	}

	/**
	 * Method used to set motors to coast when robot is disabled and on. Makes robot
	 * movement outside of the field much easier
	 * 
	 * @param idlemode Coast or brake
	 */
	// public void setIdleMode(IdleMode idlemode) {
	// SparkMaxConfig config = new SparkMaxConfig();
	// config.idleMode(idlemode);
	// drive.configure(config, SparkBase.ResetMode.kNoResetSafeParameters,
	// SparkBase.PersistMode.kPersistParameters);
	// rotate.configure(config, SparkBase.ResetMode.kNoResetSafeParameters,
	// SparkBase.PersistMode.kPersistParameters);
	// }
}
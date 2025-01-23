// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveRequest.Idle;

import frc.robot.Constants.SwerveConstants.SwerveModuleConstants.SwerveAnglePIDConstants;
import frc.robot.Constants;

/**
 * Holds methods to easily change the state of modules.
 */
public class SwerveModule extends SubsystemBase {
	private final PIDController anglePID = new PIDController(
			SwerveAnglePIDConstants.p,
			SwerveAnglePIDConstants.i,
			SwerveAnglePIDConstants.d);

	private SparkMax drive;
	private SparkMax rotate;
	private RelativeEncoder encoder;
	private CANcoder canCoder;
	private double angleOffset;

	public SwerveModule(int driveMotorID, int turnMotorID, int canCoderID, double angleOffset) { // initialize module

		drive = new SparkMax(driveMotorID, MotorType.kBrushless);
		rotate = new SparkMax(turnMotorID, MotorType.kBrushless);
		//drive.setIdleMode(IdleMode.kBrake);
		//rotate.setIdleMode(IdleMode.kBrake); //TODO: brooo

		this.canCoder = new CANcoder(canCoderID);
		encoder = drive.getEncoder();
		encoder.setPosition(0);

		anglePID.enableContinuousInput(0, Math.PI * 2);
		this.angleOffset = angleOffset;

	}

	/**
	 * Directly sets speed of drive motor based on input, and sets wheel angle with
	 * PID controller
	 * 
	 * @param state {@link SwerveModuleState} object with speed and angle
	 */
	public void setState(SwerveModuleState state) {
		state = SwerveModuleState.optimize(state, getTurningPosition()); // swerve module optimization prevents the
																			// wheel
																			// from ever turning more than 180 degrees,
																			// so it
																			// always chooses the shortest angle to
																			// rotate to

		drive.set(state.speedMetersPerSecond);
		double pidOut = anglePID.calculate(getTurningPosition().getRadians(), state.angle.getRadians());
		rotate.set(pidOut);
	}

	/**
	 * Get absolute turning position from {@link CANcoder}
	 * 
	 * @return Absolute angle of module including angleOffset
	 */
	public Rotation2d getTurningPosition() {
		return new Rotation2d((this.canCoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2 + this.angleOffset));
	}

	/**
	 * Get velocity of drive motor in m/s on ground
	 * 
	 * @return Velocity of drive motor
	 */
	public double getVelocity() { // convert rpm to m/s
		return encoder.getVelocity() * Constants.SwerveConstants.SwerveModuleConstants.conversionFactor / 60;
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
		return encoder.getPosition() * Constants.SwerveConstants.SwerveModuleConstants.conversionFactor;
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

	/**
	 * Method used to set motors to coast when robot is disabled and on. Makes robot
	 * movement outside of the field much easier
	 * 
	 * @param idlemode Coast or brake
	 */
	public void setIdleMode(IdleMode idlemode) {
		//drive.setIdleMode(idlemode); //TODO: bruh
		//rotate.setIdleMode(idlemode);
	}

}
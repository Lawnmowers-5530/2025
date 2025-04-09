// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Class with static methods to access {@link Pigeon2}. Very convenient in
 * command-based structure
 */
public class Pgyro extends SubsystemBase {
	static final class GyroConstants extends frc.robot.constants.Gyro {};
	static Rotation2d offset = new Rotation2d();
	public static Rotation2d alignOffset = new Rotation2d();

	/** Creates a new Gyro. */
	public Pgyro() {
	}

	private static final Pigeon2 pigeon = new Pigeon2(GyroConstants.pigeonPort);

	public static Pigeon2 getGyro() {
		return pigeon;
	}

	public static Rotation2d getRot() {
		return pigeon.getRotation2d().minus(offset);//.minus(Rotation2d.fromDegrees(offset));
	}

	public static Rotation2d getRawRot() {
		return pigeon.getRotation2d();
	}

	public static double getDeg() {
		//return pigeon.getYaw().getValueAsDouble();// - offset;
		return getRot().getDegrees();
	}

	public static double getRad() {
		return getDeg() * Math.PI / 180;
	}

	//public static void zeroGyro() {
	//	offset = getRot();
	//}

	public static double getHdgDeg() {
		double a;
		if (getDeg() > 0) {
			a = Math.abs(getDeg() % 360);
		} else {
			a = 360 - Math.abs(getDeg() % 360);
		}
		return a;
	}

	public static double getHdgRad() {
		double a;
		if (getRad() > 0) {
			a = Math.abs(getRad() % (Math.PI * 2));
		} else {
			a = (Math.PI * 2) - Math.abs(getRad() % (Math.PI * 2));
		}
		return a;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public static Command zeroGyroCommand() {
		return new InstantCommand(() -> {
			offset = pigeon.getRotation2d();
		});
	}

	public static Command backwardGyroCommand() {
		return new InstantCommand(() -> {
			offset = pigeon.getRotation2d().plus(Rotation2d.fromDegrees(180));
		});
	}

	public static Command setAutoGyro() {
		return new InstantCommand(
			() -> {
				alignOffset = pigeon.getRotation2d();
			}
		);
	}
	public static Command setAutoGyroWithOffset(double angle) {
		return new InstantCommand(()-> {
			alignOffset = alignOffset.minus(Rotation2d.fromDegrees(angle));
		});
	}
	public static Command zeroGyroAfterAutonCommand() {
		return new InstantCommand(()->{
			offset = getRawRot().minus(alignOffset);
		});
	}
}
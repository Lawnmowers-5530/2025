// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GyroConstants;
import frc.robot.interfaces.Gyro;

/**
 * Class with static methods to access {@link Pigeon2}. Very convenient in
 * command-based structure
 */
public class Pgyro extends SubsystemBase implements Gyro<Pigeon2> {
	/** Creates a new Gyro. */
	public Pgyro() {
	}

	private static final Pigeon2 pigeon = new Pigeon2(GyroConstants.pigeonPort);

	public static Pigeon2 getGyro() {
		return pigeon;
	}

	public static Rotation2d getRot() {
		return pigeon.getRotation2d();
	}

	public static double getDeg() {
		return pigeon.getYaw().getValueAsDouble();
	}

	public static double getRad() {
		return getDeg() * Math.PI / 180;
	}

	public static void zeroGyro() {
		pigeon.setYaw((0));
	}

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

	@Override
	public Pigeon2 interfaceGetGyro() {
		return pigeon;
	}

	@Override
	public Rotation2d interfaceGetRot() {
		return pigeon.getRotation2d();
	}

	@Override
	public double interfaceGetDeg() {
		return pigeon.getYaw().getValueAsDouble();
	}

	@Override
	public double interfaceGetRad() {
		return getDeg() * Math.PI / 180;
	}

	@Override
	public void interfaceZeroGyro() {
		pigeon.setYaw((0));
	}

	@Override
	public double interfaceGetHdgDeg() {
		double a;
		if (getDeg() > 0) {
			a = Math.abs(getDeg() % 360);
		} else {
			a = 360 - Math.abs(getDeg() % 360);
		}
		return a;
	}

	@Override
	public double interfaceGetHdgRad() {
		double a;
		if (getRad() > 0) {
			a = Math.abs(getRad() % (Math.PI * 2));
		} else {
			a = (Math.PI * 2) - Math.abs(getRad() % (Math.PI * 2));
		}
		return a;
	}

	public static Command zeroGyroCommand() {
		return new InstantCommand(() -> {
			pigeon.setYaw((0));
		});
	}
}
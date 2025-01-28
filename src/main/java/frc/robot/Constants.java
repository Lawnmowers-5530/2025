// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * Final class to hold all constants used. Keeps all code organized and makes it
 * easy to make changes to control systems
 */
public final class Constants {
	public static final boolean debug = true;

	public static final class SwerveConstants {
		public static final double trackWidth = Units.inchesToMeters(23);
		public static final double wheelBase = Units.inchesToMeters(23);

		public static final class FrontLeftModule { // FL
			public static final int driveMotor = 5;
			public static final int turnMotor = 6;
			public static final int canCoder = 13;
			public static final double angleOffset = 0;
		}

		public static final class FrontRightModule { // FR
			public static final int driveMotor = 7;
			public static final int turnMotor = 8;
			public static final int canCoder = 14;
			public static final double angleOffset = 0;
		}

		public static final class RearRightModule { // RR
			public static final int driveMotor = 9;
			public static final int turnMotor = 10;
			public static final int canCoder = 15;
			public static final double angleOffset = 0;
		}

		public static final class RearLeftModule { // RL
			public static final int driveMotor = 11;
			public static final int turnMotor = 12;
			public static final int canCoder = 16;
			public static final double angleOffset = 0;
		}

		public static final class SwerveModuleConstants {
			public static final double conversionFactor = (1 / 6.75) * Units.inchesToMeters(Math.PI * 4);

			/**
			 * number to multiply
			 * rotations
			 * by to
			 * find meters of movement, specifically rpm to m/min conversion based on wheel size
			 */

			public static final class SwerveAnglePIDConstants {
				public static final double p = 0.3;
				public static final double i = 0.125;
				public static final double d = 0;
			}
		}

		public static final class RotationConstants {
			public static final double kP = 0.75;
			public static final double kI = 0.0;
			public static final double kD = 0.0;

			public static final double controllerTolerance = 0.1; // Arbitrary PIDController tolerance to determine when
																	// setpoint is reached. Relevant for knowing when
																	// rotation based commands finish
		}

		public static final class PathPlannerConstants {
			public static final double driveBaseRadius = Units
					.inchesToMeters(Math.sqrt(trackWidth * trackWidth + wheelBase * wheelBase));

			public static final PIDConstants translationConstants = new PIDConstants(5, 0.5, 0);
			public static final PIDConstants rotationConstants = new PIDConstants(3, 0, 0);

			public static final PathConstraints constraints = new PathConstraints(1, 1, 1, 1);
		}

		public static final Translation2d front_left = new Translation2d(Constants.SwerveConstants.trackWidth / 2,
				Constants.SwerveConstants.wheelBase / 2);
		public static final Translation2d front_right = new Translation2d(Constants.SwerveConstants.trackWidth / 2,
				-Constants.SwerveConstants.wheelBase / 2);
		public static final Translation2d rear_left = new Translation2d(-Constants.SwerveConstants.trackWidth / 2,
				Constants.SwerveConstants.wheelBase / 2);
		public static final Translation2d rear_right = new Translation2d(-Constants.SwerveConstants.trackWidth / 2,
				-Constants.SwerveConstants.wheelBase / 2); //RR
		public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(front_left, front_right, rear_right, rear_left);
	}

	public static final class ControllerConstants {
		public static final int driveControllerPort = 0;
		public static final double driveControllerJoystickDeadband = 0.06;

		public static final int secondaryControllerPort = 1;
		public static final double secondaryControllerJoystickDeadband = 0.1;
	}

	public static final class VisionTargeterConstants {
		public static final double kP = 0.1;
		public static final double kI = 0.1;
		public static final double kD = 0.1;

		public static final double taMultiplier = 1.25;
	}

	public static final class GyroConstants {
		public static final int pigeonPort = 17;
	}

	public static final class LimelightConstants {
		public static final String primaryTableKeyName = "limelight";
	}
}

package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
public class Swerve {
    public static final double fieldWidth = Units.feetToMeters(26) + Units.inchesToMeters(5);
    public static final double fieldLength = Units.feetToMeters(57) + Units.inchesToMeters(6.875);

    public static final double trackWidth = Units.inchesToMeters(24);
    public static final double wheelBase = Units.inchesToMeters(24);

    public static final double leftStationAngle = 0;
    public static final double rightStationAngle = 0;

    public static final class FrontLeftModule { // FL
        public static final int driveMotor = 5;
        public static final int turnMotor = 6;
        public static final int canCoder = 13;
        public static final double angleOffset = 0;
        public static final boolean inverted = true;
    }

    public static final class FrontRightModule { // FR
        public static final int driveMotor = 7;
        public static final int turnMotor = 8;
        public static final int canCoder = 14;
        public static final double angleOffset = 0.1;
        public static final boolean inverted = true;
    }

    public static final class RearRightModule { // RR
        public static final int driveMotor = 9;
        public static final int turnMotor = 10;
        public static final int canCoder = 15;
        public static final double angleOffset = 0;
        public static final boolean inverted = true;
    }

    public static final class RearLeftModule { // RL
        public static final int driveMotor = 11;
        public static final int turnMotor = 12;
        public static final int canCoder = 16;
        public static final double angleOffset = 0;
        public static final boolean inverted = false;
    }

    public static class SwerveModule {
        public static final double conversionFactor = (1 / 6.12) * Units.inchesToMeters(Math.PI * 4);

        /**
         * number to multiply
         * rotations
         * by to
         * find meters of movement, specifically rpm to m/min conversion based on wheel size
         */

        public static class SwerveAnglePIDConstants {
            public static final double p = 0.3;
            public static final double i = 0.125;
            public static final double d = 0;
        }

        public static final double slowModeScaleFactor = (double) 1 / 3;
    }

    public static class Rotation {
        public static final double kP = 0.75;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double controllerTolerance = 0.1; // Arbitrary PIDController tolerance to determine when
        // setpoint is reached. Relevant for knowing when
        // rotation based commands finish
    }

    public static class PathPlanner {
        public static final double driveBaseRadius = Units
                .inchesToMeters(Math.sqrt(trackWidth * trackWidth + wheelBase * wheelBase));

        public static final PIDConstants translationConstants = new PIDConstants(4, 0, 0);
        public static final PIDConstants rotationConstants = new PIDConstants(1, 0, 0);

        public static final PathConstraints constraints = new PathConstraints(1, 1, 1, 1);
    }

    public static final Translation2d front_left = new Translation2d(trackWidth / 2,
            wheelBase / 2);
    public static final Translation2d front_right = new Translation2d(trackWidth / 2,
            -wheelBase / 2);
    public static final Translation2d rear_left = new Translation2d(-trackWidth / 2,
            wheelBase / 2);
    public static final Translation2d rear_right = new Translation2d(-trackWidth / 2,
            -wheelBase / 2); //RR
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(front_left, front_right, rear_right, rear_left);
}

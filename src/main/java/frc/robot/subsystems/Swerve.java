// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import io.github.oblarg.oblog.Loggable;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.Container;
import frc.robot.subsystems.vision.PoseCameraManager;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Drivetrain control subsystem. Uses {@link SwerveModule}s to control the
 * movement of the robot. Also handles autonomous routines from PathPlanner.
 */
public class Swerve extends SubsystemBase implements Loggable {

	// impart frc.robot.constants.Swerve as SwerveConstants
	static final class SwerveConstants extends frc.robot.constants.Swerve {
	};

	// import frc.robot.constants.AlignToTag as AlignConstants
	static final class AlignConstants extends frc.robot.constants.AlignToTag {
	};

	private PoseCameraManager cameraManager;
	private PIDController rotationPID;

	private SwerveDrivePoseEstimator odometry;

	private SwerveModule frontLeftModule;
	private SwerveModule frontRightModule;
	private SwerveModule rearRightModule;
	private SwerveModule rearLeftModule;

	@Log
	Pose2d currentPose;

	RobotConfig config;

	/**
	 * Initialize all swerve elements
	 */
	public Swerve() {
		currentPose = new Pose2d();

		this.frontLeftModule = new SwerveModule(
				SwerveConstants.FrontLeftModule.driveMotor,
				SwerveConstants.FrontLeftModule.turnMotor,
				SwerveConstants.FrontLeftModule.canCoder,
				SwerveConstants.FrontLeftModule.angleOffset,
				SwerveConstants.FrontLeftModule.inverted);

		this.frontRightModule = new SwerveModule(
				SwerveConstants.FrontRightModule.driveMotor,
				SwerveConstants.FrontRightModule.turnMotor,
				SwerveConstants.FrontRightModule.canCoder,
				SwerveConstants.FrontRightModule.angleOffset,
				SwerveConstants.FrontRightModule.inverted);
		this.rearRightModule = new SwerveModule(
				SwerveConstants.RearRightModule.driveMotor,
				SwerveConstants.RearRightModule.turnMotor,
				SwerveConstants.RearRightModule.canCoder,
				SwerveConstants.RearRightModule.angleOffset,
				SwerveConstants.RearRightModule.inverted);
		this.rearLeftModule = new SwerveModule(
				SwerveConstants.RearLeftModule.driveMotor,
				SwerveConstants.RearLeftModule.turnMotor,
				SwerveConstants.RearLeftModule.canCoder,
				SwerveConstants.RearLeftModule.angleOffset,
				SwerveConstants.RearLeftModule.inverted);
		rotationPID = new PIDController(
				SwerveConstants.Rotation.kP,
				SwerveConstants.Rotation.kI,
				SwerveConstants.Rotation.kD);
		rotationPID.setTolerance(SwerveConstants.Rotation.controllerTolerance); // useful to tell commands when
																				// the
																				// target angle has been
																				// reached
		SwerveModulePosition[] modPos = getModulePositions();

		this.cameraManager = new PoseCameraManager();

		odometry = new SwerveDrivePoseEstimator(
				SwerveConstants.kinematics,
				Pgyro.getRot(),
				modPos,
				getPose(),
				VecBuilder.fill(0.1, 0.1, 0.1), // set state std devs to 0.1,0.1,0.1 - these are //TODO
												// relative to
												// the vision devs and only determine the
												// magnitude of the vision devs

				VecBuilder.fill(100, 100, 100)); // initialize vision std devs to 100,100,100 to
													// allow
													// limelight feedback to set devs

		try {
			config = RobotConfig.fromGUISettings();
		} catch (Exception e) {
			// Handle exception as needed
			e.printStackTrace();
		}

		AutoBuilder.configure(
				this::getPose,
				this::resetPose,
				this::getRobotRelativeSpeeds,
				(speeds, feedforwards) -> autoDriveRobotRelative(speeds),
				new PPHolonomicDriveController(
						SwerveConstants.PathPlanner.translationConstants,
						SwerveConstants.PathPlanner.rotationConstants),
				config,
				() -> {
					// BooleanSupplier that controls when the path will be mirrored for the red
					// alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE
					Optional<Alliance> alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
				this);
	}

	/**
	 * Uses Pathplanner to generate path from current position to endPose
	 * 
	 * @param endPose Goal robot {@link Pose2d}
	 * @return {@link Command} to pathfind to given Pose2d
	 */
	public Command pathFind(Pose2d endPose) {
		return AutoBuilder.pathfindToPose(
				endPose,
				SwerveConstants.PathPlanner.constraints);
	}

	/**
	 * Repeat call drive with supplier outputs
	 * 
	 * @param vector
	 * @param omegaRadSecSupplier
	 * @param fieldRelative
	 * @param slowModeSupplier    A {@link Trigger} from a button to supply a
	 *                            boolean for
	 *                            slow mode
	 * @return {@link RunCommand} to drive with suppliers
	 */
	public Command drive() {
		return new RunCommand(
				() -> {
					this.drive(Controller.driveVector.get(), Controller.driveRotation.get(), true,
							Controller.slowMode.getAsBoolean() ? 0.5 : 1);
				}, this);
	};

	/**
	 * 
	 * @param vector        {@link Vector2D} to drive to. Uses the FRC coordinate
	 *                      system:
	 *                      i-hat [0,1],
	 *                      j-hat [-1,0]
	 * @param omegaRadSec   CCW+ radians per second
	 * @param fieldRelative Determines if vector is driven relative to 0 deg gyro
	 *                      angle
	 * @param scalar        A double to scale all inputs by
	 */
	public void drive(Vector<N2> vector, double omegaRadSec, boolean fieldRelative, double scalar) {
		SmartDashboard.putString("Vec: ", vector.toString());
		vector = vector.times(scalar);
		omegaRadSec = omegaRadSec * scalar;

		ChassisSpeeds speeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
				new ChassisSpeeds(
						vector.get(0),
						vector.get(1),
						omegaRadSec),
				Pgyro.getRot())
				: new ChassisSpeeds(
						vector.get(0),
						vector.get(1),
						omegaRadSec);

		SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(speeds);
		frontLeftModule.setState(states[0]);
		frontRightModule.setState(states[1]);
		rearRightModule.setState(states[2]);
		rearLeftModule.setState(states[3]);
		SmartDashboard.putString("rr", states[2].toString());

	}

	/**
	 * Uses {@link PIDController} to reach rotation target
	 * 
	 * @param vectorSupplier Vector {@link Supplier} to continue using controller
	 *                       input for
	 *                       movement
	 * @param yawSupplier    {@link DoubleSupplier} to tell the controller what the
	 *                       current
	 *                       goal yaw is
	 * @param scalar         Scalar applied to the vector only
	 * @return A {@link RunCommand} to rotate to the angle
	 */
	public Command yawController(Supplier<Vector<N2>> vectorSupplier, DoubleSupplier yawSupplier, double scalar) {
		return new RunCommand(
				() -> {
					double rotationOutput = rotationPID.calculate(Pgyro.getRot().getRadians(),
							yawSupplier.getAsDouble());
					this.drive(vectorSupplier.get().times(scalar), rotationOutput, false, 1);
				},
				this).until(() -> {
					return rotationPID.atSetpoint(); // lambda boolean supplier to detect if at rotation setpoint
				});
	}

	/**
	 * Robot relative drive {@link ChassisSpeeds} consumer for pathplanner
	 * 
	 * @param speeds ChassisSpeeds object
	 */
	public void autoDriveRobotRelative(ChassisSpeeds speeds) {
		this.drive(
				VecBuilder.fill(
						speeds.vxMetersPerSecond,
						speeds.vyMetersPerSecond),
				speeds.omegaRadiansPerSecond,
				false, 1);
	}

	/**
	 * Send {@link SwerveModuleState}s to SwerveModule objects
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		frontLeftModule.setState(desiredStates[0]);
		frontRightModule.setState(desiredStates[1]);
		rearRightModule.setState(desiredStates[2]);
		rearLeftModule.setState(desiredStates[3]);
	}

	/**
	 * Method primarily for PathPlanner to reset robot position in odometry
	 * 
	 * @param pose The {@link Pose2d} to reset to
	 */
	public void resetPose(Pose2d pose) {
		odometry.resetPosition(
				Pgyro.getRot(),
				getModulePositions(),
				pose);
	}

	/**
	 * Updates odometry and disables coasting if robot enables
	 */
	@Override
	public void periodic() {
		updateOdometry();
		currentPose = getPose();
		SmartDashboard.putString("current pose:", this.currentPose.toString());
		SmartDashboard.putString("robot rel speed", this.robotRelativeSpeedString());
	}

	/**
	 * Fetch List of optional estimated poses paired with calculated std devs from
	 * each pose camera and add a vision measurement for each present optional
	 */
	public void updateOdometry() {
		odometry.update(
				Pgyro.getRot(),
				getModulePositions());

		var visionEstimates = cameraManager.getEstimatedPoses();

		for (var visionEstimate : visionEstimates) {
			var estimate = visionEstimate.getFirst();
			var deviations = visionEstimate.getSecond();
			odometry.addVisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, deviations);
		}
	}

	/**
	 * Controls if motors are in coast or brake mode depending on if robot is
	 * enabled
	 */
	public void disabledPeriodic() {
		// if (!isCoasting) {
		// isCoasting = true;
		// frontLeftModule.setIdleMode(IdleMode.kCoast);
		// frontRightModule.setIdleMode(IdleMode.kCoast); //TODO: wtf
		// rearRightModule.setIdleMode(IdleMode.kCoast);
		// rearLeftModule.setIdleMode(IdleMode.kCoast);
		// }
	}

	/**
	 * @return String representation of robot pose
	 */
	public String poseString() {
		return getPose().toString();
	}

	/**
	 * 
	 * @return String representation of robot relative speed components
	 */
	public String robotRelativeSpeedString() {
		return getRobotRelativeSpeeds().toString();
	}

	/**
	 * Returns current robot pose from odometry
	 * 
	 * @return Robot {@link Pose2d} from odometry
	 */
	public Pose2d getPose() {

		// this probably seems dumb, but somehow this gets called before odometry exists
		// soo...
		if (odometry == null) {
			return new Pose2d();
		}

		return odometry.getEstimatedPosition();
	}

	/**
	 * 
	 * @return {@link SwerveModulePosition} array of each module position
	 */
	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {
				frontLeftModule.getPos(),
				frontRightModule.getPos(),
				rearRightModule.getPos(),
				rearLeftModule.getPos(),
		};
	}

	/**
	 * Returns robot relative {@link ChassisSpeeds}, designed for PathPlanner
	 * control
	 * 
	 * @return Robot relative chassis speeds object
	 */
	public ChassisSpeeds getRobotRelativeSpeeds() {
		return SwerveConstants.kinematics.toChassisSpeeds(
				frontLeftModule.getState(),
				frontRightModule.getState(),
				rearRightModule.getState(),
				rearLeftModule.getState());
	}

	public class AlignToTag extends Command {
		static final double tagAmbiguityThreshold = 0.2;
		PIDController yawPID = new PIDController(AlignConstants.kProt, AlignConstants.kIrot,
				AlignConstants.kDrot);
		PIDController drivePID = new PIDController(AlignConstants.kPtrans, AlignConstants.kItrans,
				AlignConstants.kDtrans);
		boolean left;
		double y;
		double x;
		Rotation2d rot;
		Transform3d cameraToRobot;

		public AlignToTag(boolean left) {
			this.y = 0;
			this.x = 0;
			this.rot = new Rotation2d();

			this.left = left;
			yawPID.setSetpoint(180);
			yawPID.setIZone(2);
			yawPID.enableContinuousInput(-180, 180);
			drivePID.setSetpoint(0);
			drivePID.setTolerance(0.02);
			yawPID.setTolerance(1);
			
			addRequirements(Swerve.this);
		}

		@Override
		public void execute() {
			ArrayList<PhotonTrackedTarget> tags;
			if (this.left) {
				tags = cameraManager.getTagsById(cameraManager.getPrimaryIdLeft());
				cameraToRobot = AlignConstants.leftCameraToRobot;
			} else {
				tags = cameraManager.getTagsById(cameraManager.getPrimaryIdRight());
				cameraToRobot = AlignConstants.rightCameraToRobot;
			}
			// sort tags by the tag's pose ambiguity
			if (tags.isEmpty()) {
				this.cancel();
			}
			var tracked_tag = tags
					.stream()
					.filter(tag -> tag.getPoseAmbiguity() != -1 && tag.getPoseAmbiguity() < 0.2)
					.min(Comparator.comparingDouble(PhotonTrackedTarget::getPoseAmbiguity));

			tracked_tag.ifPresent(
					tag -> {
						Transform3d camTrans = tag.getBestCameraToTarget();
						System.out.println(camTrans.getRotation().toRotation2d().getDegrees());
						Pose3d estimate = PhotonUtils.estimateFieldToRobotAprilTag(camTrans,
								new Pose3d(0, 0, 0.2, new Rotation3d()), cameraToRobot);
						rot = estimate.getRotation().toRotation2d();
						y = estimate.getTranslation().getY();
						x = estimate.getTranslation().getX();
					});
			Swerve.this.autoDriveRobotRelative(new ChassisSpeeds(-drivePID.calculate(x),
					-drivePID.calculate(y), yawPID.calculate(rot.getDegrees())));
		}
	}

	public void setSlowMode(boolean slowMode) {
		frontLeftModule.setSlowMode(slowMode);
		frontRightModule.setSlowMode(slowMode);
		rearRightModule.setSlowMode(slowMode);
		rearLeftModule.setSlowMode(slowMode);
	}
}
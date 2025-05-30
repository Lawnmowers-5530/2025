// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.vision.PoseCameraManager;
import io.github.oblarg.oblog.Loggable;
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

	static final class RobotGlobalConstants extends frc.robot.constants.RobotGlobal {};

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
				Pgyro.getRawRot(),
				modPos,
				Pose2d.kZero, // pathplanner overrides this. Make sure to reset this if not using pathplanner
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
				(speeds, feedforwards) -> autoDriveRobotRelative(speeds, 1/4.8),
				new PPHolonomicDriveController(
						SwerveConstants.PathPlanner.translationConstants,
						SwerveConstants.PathPlanner.rotationConstants, RobotGlobalConstants.timePeriodS),
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
				this);
	}

	/**
	 * Robot relative drive {@link ChassisSpeeds} consumer for pathplanner
	 * 
	 * @param speeds ChassisSpeeds object
	 */
	public void autoDriveRobotRelative(ChassisSpeeds speeds, double scalar) {
		this.drive(
				VecBuilder.fill(
						speeds.vxMetersPerSecond,
						speeds.vyMetersPerSecond),
				speeds.omegaRadiansPerSecond,
				false, scalar);
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
				Pgyro.getRawRot(),
				getModulePositions(),
				pose);
		// Pgyro.setAutoGyro();

	}

	public Command resetPoseWithVision() {
		return new InstantCommand(()-> {
			var visionEstimates = cameraManager.getEstimatedPoses();
			if (visionEstimates.size() < 2){
				System.out.print("Failed to see targets");
				return;
			}
			double xSum = 0;
			double ySum = 0;
			double rotSum = 0;
			int count = 0;
			for (var visionEstimate : visionEstimates) {
				var estimate = visionEstimate.getFirst().estimatedPose.toPose2d();
				xSum += estimate.getX();
				ySum += estimate.getY();
				rotSum += estimate.getRotation().getRadians();
				count ++;
				
			}
			xSum/=count;
			ySum/=count;
			rotSum/=count;
			odometry.resetPose(new Pose2d(xSum, ySum, new Rotation2d(rotSum)));
		});

	}

	/**
	 * Updates odometry and disables coasting if robot enables
	 */
	@Override
	public void periodic() {
		SmartDashboard.putNumber("gyro deg:", Pgyro.getDeg());
		updateOdometry();
		currentPose = getPose();
		SmartDashboard.putString("current pose:", this.currentPose.toString());
		//SmartDashboard.putString("bl out", rearLeftModule.getPos().toString());
		//SmartDashboard.putNumber("speed", (this.frontLeftModule.getVelocity() + this.frontRightModule.getVelocity()
		//		+ this.rearLeftModule.getVelocity() + this.rearRightModule.getVelocity()) / 4);
		//SmartDashboard.putNumber("Auto Gyro Offset", Pgyro.alignOffset.getDegrees());
		//SmartDashboard.putNumber("Gyro measurement raw", Pgyro.getRawRot().getDegrees());
		//SmartDashboard.putNumber("Gyro measurement", Pgyro.getRot().getDegrees());
		SmartDashboard.putBoolean("left cam status: ", cameraManager.camList.get(0).hasTargets());
		SmartDashboard.putBoolean("right cam status: ", cameraManager.camList.get(1).hasTargets());
	}

	/**
	 * Fetch List of optional estimated poses paired with calculated std devs from
	 * each pose camera and add a vision measurement for each present optional
	 */
	public void updateOdometry() {
		odometry.update(
				Pgyro.getRawRot(),
				getModulePositions());

		/*var visionEstimates = cameraManager.getEstimatedPoses();

		for (var visionEstimate : visionEstimates) {
			var estimate = visionEstimate.getFirst();
			var deviations = visionEstimate.getSecond();
			odometry.addVisionMeasurement(cameraManager.flipPose(estimate.estimatedPose.toPose2d()),
			estimate.timestampSeconds, deviations);
		}*/
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

	public class AlignToTagRight extends Command {
		static final double tagAmbiguityThreshold = 0.2;
		PIDController yawPID = new PIDController(AlignConstants.kProt, AlignConstants.kIrot,
				AlignConstants.kDrot);
		PIDController xdrivePID = new PIDController(AlignConstants.xkPtrans, AlignConstants.xkItrans,
				AlignConstants.xkDtrans);
		PIDController ydrivePID = new PIDController(AlignConstants.ykPtrans, AlignConstants.ykItrans,
				AlignConstants.ykDtrans);
		double y;
		double x;
		double yaw;
		double yawTarget;
		Rotation2d rot;
		Transform3d cameraToRobot;
		boolean auton;
		boolean isPoseSet = false;
		SwerveDrivePoseEstimator pose_est;

		@Override
		public void initialize() {
			isPoseSet = false;
		}

		public AlignToTagRight(boolean auton) {
			this.auton = auton;
			yawTarget = 0;

			this.y = 0;
			this.x = 0;
			this.rot = new Rotation2d();
			
			yawPID.setSetpoint(180);
			yawPID.setIZone(2);
			yawPID.enableContinuousInput(-180, 180);
			yawPID.setTolerance(AlignConstants.rotatationTolerance);

			xdrivePID.setSetpoint(0);
			ydrivePID.setSetpoint(0);

			xdrivePID.setTolerance(AlignConstants.xDriveTolerance);
			ydrivePID.setTolerance(AlignConstants.yDriveTolerance);
			pose_est = new SwerveDrivePoseEstimator(SwerveConstants.kinematics, Pgyro.getRot(), getModulePositions(), Pose2d.kZero);
			addRequirements(Swerve.this);
		}

		public boolean isFinished() {
			return yawPID.atSetpoint() && xdrivePID.atSetpoint() && ydrivePID.atSetpoint();
		}

		
		

		@Override
		public void execute() {
			if (isFinished()) {
				Controller.rumbleLeft = true;
			}
			pose_est.update(Pgyro.getRot(), getModulePositions());
			yaw = Pgyro.getDeg();
			if (auton) {
				yaw = Pgyro.getRawRot().getDegrees();
				yaw = yaw - Pgyro.alignOffset.getDegrees();
			}
			Optional<PhotonTrackedTarget> tags;
			tags = cameraManager.getPrimaryTargetRight();
			cameraToRobot = AlignConstants.rightCameraToRobot;
			Optional<PhotonTrackedTarget> tracked_tag;
			if (tags.isEmpty()) {
				tracked_tag= Optional.empty();
			}else {
				tracked_tag = tags.get().getPoseAmbiguity() != -1
					&& tags.get().getPoseAmbiguity() < 0.2 ? Optional.of(tags.get()) : Optional.empty();

			}
			// sort tags by the tag's pose ambiguity
			x = pose_est.getEstimatedPosition().getX();
			y = pose_est.getEstimatedPosition().getY();
			tracked_tag.ifPresent(
					tag -> {
						cameraToRobot = AlignConstants.tagOffsets.containsKey(tag.getFiducialId())
								? cameraToRobot.plus(AlignConstants.tagOffsets.get(tag.getFiducialId()).inverse()) // TODO
																													// check
																													// functionality
								: cameraToRobot;
						Transform3d camTrans = tag.getBestCameraToTarget();
						SmartDashboard.putString("camTrans", camTrans.toString());
						Pose3d estimate = PhotonUtils.estimateFieldToRobotAprilTag(camTrans,
								new Pose3d(0, 0, 0.275, new Rotation3d()), cameraToRobot);
						if (!isPoseSet) {
							isPoseSet = true;
							pose_est.resetPose(estimate.toPose2d());
						}
						xdrivePID.setP(Math.max(0, AlignConstants.xkPtrans - 0.35 * Math.abs(ydrivePID.getError())));
						SmartDashboard.putNumber("xdriveP", AlignConstants.xkPtrans - 0.35 * ydrivePID.getError());
						rot = estimate.getRotation().toRotation2d();
						y = estimate.getTranslation().getY();
						x = estimate.getTranslation().getX();

						SmartDashboard.putNumber("xVal", x);
						SmartDashboard.putNumber("yVal", y);
						if (AlignConstants.useGyro) {
							yawTarget = getTagAngle(tag.getFiducialId());
							
							SmartDashboard.putNumber("Target Yaw Align", yawTarget);
						} else {
							yawTarget = 180;
							yaw = rot.getDegrees();

						}

					});
			if (tracked_tag.isEmpty() && !isPoseSet) {
				return;
			}
			Swerve.this.autoDriveRobotRelative(new ChassisSpeeds(-xdrivePID.calculate(x),
					-ydrivePID.calculate(y), yawPID.calculate(yaw, yawTarget)), 1);
		}
	}

	public class AlignToTagLeft extends Command {
		static final double tagAmbiguityThreshold = 0.2;
		PIDController yawPID = new PIDController(AlignConstants.kProt, AlignConstants.kIrot,
				AlignConstants.kDrot);
		PIDController xdrivePID = new PIDController(AlignConstants.xkPtrans, AlignConstants.xkItrans,
				AlignConstants.xkDtrans);
		PIDController ydrivePID = new PIDController(AlignConstants.ykPtrans, AlignConstants.ykItrans,
				AlignConstants.ykDtrans);
		double y;
		double x;
		double yaw;
		double yawTarget;
		Rotation2d rot;
		Transform3d cameraToRobot;
		boolean auton;
		SwerveDrivePoseEstimator pose_est;
		boolean setInitPose = false;
		/*
		 * Best practice is to just work completely in field coordinates
		 */
		

		public AlignToTagLeft(boolean auton) {
			this.auton = auton;
			yawTarget = 0;

			this.y = 0;
			this.x = 0;
			this.rot = new Rotation2d();
			pose_est = new SwerveDrivePoseEstimator(SwerveConstants.kinematics, Pgyro.getRot(), getModulePositions(), Pose2d.kZero);

			yawPID.setSetpoint(180);
			yawPID.setIZone(2);
			yawPID.enableContinuousInput(-180, 180);
			yawPID.setTolerance(AlignConstants.rotatationTolerance);

			xdrivePID.setSetpoint(0);
			ydrivePID.setSetpoint(0);

			xdrivePID.setTolerance(AlignConstants.xDriveTolerance);
			ydrivePID.setTolerance(AlignConstants.yDriveTolerance);

			addRequirements(Swerve.this);
			
		}

		public boolean isFinished() {
			return yawPID.atSetpoint() && xdrivePID.atSetpoint() && ydrivePID.atSetpoint();
		}
		
		@Override
		public void initialize() {
			setInitPose = false;
			
		}
		@Override
		public void execute() {

			SmartDashboard.putString("pose est", pose_est.getEstimatedPosition().toString());
			SmartDashboard.putNumber("gyro auton", Pgyro.alignOffset.getDegrees());
			if (isFinished()) {
				Controller.rumbleRight = true;
			}
			pose_est.update(Pgyro.getRot(), getModulePositions());
			yaw = Pgyro.getDeg();
			if (auton) {
				yaw = Pgyro.getRawRot().getDegrees();
				yaw -= Pgyro.alignOffset.getDegrees();
			}


			Optional<PhotonTrackedTarget> tags;
			tags = cameraManager.getPrimaryTargetLeft();
			cameraToRobot = AlignConstants.leftCameraToRobot;
			
			Optional<PhotonTrackedTarget> tracked_tag;
			// sort tags by the tag's pose ambiguity
			//DELETE THIS???
			if (tags.isEmpty()) {
				tracked_tag = Optional.empty();
			}else {
				tracked_tag = tags.get().getPoseAmbiguity() != -1
					&& tags.get().getPoseAmbiguity() < 0.2 ? Optional.of(tags.get()) : Optional.empty();
			}
			
			
			x = pose_est.getEstimatedPosition().getX();
			y = pose_est.getEstimatedPosition().getY();
			

			tracked_tag.ifPresent(
					tag -> {
					
						cameraToRobot = AlignConstants.tagOffsets.containsKey(tag.getFiducialId())
								? cameraToRobot.plus(AlignConstants.tagOffsets.get(tag.getFiducialId()))
								: cameraToRobot;

						Transform3d camTrans = tag.getBestCameraToTarget();
						SmartDashboard.putString("camTrans", camTrans.toString());
						Pose3d estimate = PhotonUtils.estimateFieldToRobotAprilTag(camTrans,
						new Pose3d(0, 0, -0.275, new Rotation3d()), cameraToRobot);
						if (!setInitPose) {
							setInitPose = true;
							pose_est.resetPose(estimate.toPose2d());
						}else {
							pose_est.addVisionMeasurement(estimate.toPose2d(), Timer.getFPGATimestamp()); // TO be tested
						}

						
						//Make sure things are fine here (such that y is the correct thing in the plus)
						

						xdrivePID.setP(Math.max(0, AlignConstants.xkPtrans - 0.35 * Math.abs(ydrivePID.getError())));
						SmartDashboard.putNumber("xdriveP", AlignConstants.xkPtrans - 0.35 * ydrivePID.getError());
						rot = estimate.getRotation().toRotation2d();
						y = estimate.getTranslation().getY();
						x = estimate.getTranslation().getX();

						SmartDashboard.putNumber("xVal", x);
						SmartDashboard.putNumber("yVal", y);
						if (AlignConstants.useGyro) {
							yawTarget = getTagAngle(tag.getFiducialId());
							SmartDashboard.putNumber("Target Yaw Align", yawTarget);
						} else {
							yawTarget = 180;
							yaw = rot.getDegrees();

						}
						

					});
			if (tags.isEmpty() && !setInitPose) {
				return;
			}
			
			SmartDashboard.putNumber("Auto Align Yaw Post", yaw);
			SmartDashboard.putNumber("What the fuck it should be", Pgyro.getRawRot().getDegrees() - Pgyro.alignOffset.getDegrees());
			SmartDashboard.putNumber("Yaw Pid Out", yawPID.calculate(yaw, yawTarget));
			Swerve.this.autoDriveRobotRelative(new ChassisSpeeds(-xdrivePID.calculate(x),
					-ydrivePID.calculate(y), yawPID.calculate(yaw, yawTarget)), 1);
		}
	}

	public void setSlowMode(boolean slowMode) {
		frontLeftModule.setSlowMode(slowMode);
		frontRightModule.setSlowMode(slowMode);
		rearRightModule.setSlowMode(slowMode);
		rearLeftModule.setSlowMode(slowMode);
	}


	public int getTagAngle(int id) {
		return switch (id) {
			case 6 -> -60;
			case 7 -> 0;
			case 8 -> 60;
			case 9 -> 120;
			case 10 -> 180;
			case 11 -> -120;
			case 18 -> 0;
			case 19 -> -60;
			case 20 -> -120;
			case 21 -> 180;
			case 22 -> 120;
			case 17 -> 60;

			default -> 0;
		};
	}
	public class RotateToFaceCoralStation extends Command {
		int side;
		boolean fieldRelative;
		PIDController yawPID;

		public static final int RIGHT = 60;
		public static final int LEFT = -60;


		public RotateToFaceCoralStation(int side, boolean fieldRelative) {
			this.addRequirements(Swerve.this);
			this.side = side;
			this.fieldRelative = fieldRelative;
			yawPID  = new PIDController(AlignConstants.kProt, AlignConstants.kIrot,
			AlignConstants.kDrot);
			yawPID.setIZone(2);
			yawPID.enableContinuousInput(-180, 180);
			yawPID.setTolerance(AlignConstants.rotatationTolerance);
		}

		@Override
		public void execute() {
			SmartDashboard.putNumber("rot err", Pgyro.getDeg() - side);
			SmartDashboard.putNumber("auto rot out", yawPID.calculate(Pgyro.getDeg(), side));
			Swerve.this.drive(Controller.driveVector.get(), 
			yawPID.calculate(Pgyro.getDeg(), side), 
			fieldRelative,
			1);
		}
		@Override
		public boolean isFinished() {
			return false;
		}




		
	}
}
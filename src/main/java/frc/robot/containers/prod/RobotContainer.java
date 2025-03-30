// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.containers.prod;

import java.util.Objects;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Bonk;
import frc.robot.subsystems.Bonk.Targets;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.LedManager;
import frc.robot.subsystems.Pgyro;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.RotateToFaceCoralStation;
import io.github.oblarg.oblog.Logger;

/**
 * The {@link RobotContainer} holds all subsystems, commands, suppliers, etc. in
 * a command-based structure. The only action in {@link Robot} is to schedule
 * the commands defined here, and to create an instance of the
 * {@link RobotContainer}, which will also bind all buttons to their respective
 * commands.
 */
public class RobotContainer {
	private SendableChooser<Command> autoChooser;

	public class Subsystems {
		public Swerve swerve;
		public Controller controller;
		public CoralIntake coralIntake;
		public Elevator elevator;
		public Hang hang;
		public LedManager ledManager;
		public Bonk bonk;

		// Watch for this
		public AlgaeIntake algaeIntake;

	}

	public class State {
	}

	Controller controller = new Controller();

	private Subsystems subsystems;
	private Bindings bindings;

	public RobotContainer() {

		Logger.configureLoggingAndConfig(this, false);

		CameraServer.startAutomaticCapture();

		/**
		 * initalize subsystems here
		 */
		{

			this.subsystems = new Subsystems();
			this.subsystems.ledManager = new LedManager(this.subsystems, 2);
			this.subsystems.hang = new Hang();
			this.subsystems.controller = new Controller();
			this.subsystems.coralIntake = new CoralIntake();
			this.subsystems.elevator = new Elevator();
			this.subsystems.bonk = new Bonk(); 
			this.subsystems.swerve = new Swerve();

			// if (this.subsystems.algaeIntake == null) {
			// throw new IllegalStateException("This Code is Not Commented. Algae Intake is
			// going to be Initialized");
			// }
			// this.subsystems.algaeIntake = new AlgaeIntake();
			// the death zone??
		}

		/**
		 * bind commands here
		 */
		{
			this.bindings = new Bindings(this.subsystems);

			this.subsystems.swerve.setDefaultCommand(this.subsystems.swerve.drive());
			
			//this.subsystems.swerve.setDefaultCommand(
			//	new RunCommand(
			//		() -> {
			//			this.subsystems.swerve.autoDriveRobotRelative(new ChassisSpeeds(Controller.driveVector.get().get(0), Controller.driveVector.get().get(1), -Controller.driveRotation.get()));
			//		}, this.subsystems.swerve)
			//);

			Controller.L1.onTrue(this.bindings.elevator.goToL1());
			Controller.L2.onTrue(this.bindings.elevator.goToL2());
			Controller.L3.onTrue(this.bindings.elevator.goToL3());
			Controller.L4.onTrue(this.bindings.elevator.goToL4());

			Controller.zeroGyro.onTrue(this.bindings.swerve.zeroGyro());

			Controller.intake.onTrue(this.bindings.coral.runIntake());
			Controller.outtake.onTrue(this.bindings.coral.conditionalOuttake());

			

			Controller.alignLeft.whileTrue(this.subsystems.swerve.new AlignToTagLeft(false));
			Controller.alignLeft.onFalse(new InstantCommand(
					() -> {
						Controller.rumbleLeft = false;
					}));
			Controller.alignRight.whileTrue(this.subsystems.swerve.new AlignToTagRight(false));
			Controller.alignRight.onFalse(new InstantCommand(
					() -> {
						Controller.rumbleRight = false;
					}));

			Controller.toggleLaserCan.onChange(new RunCommand(() -> {
				this.subsystems.coralIntake.setLaserCanSwitch(Controller.toggleLaserCan.getAsBoolean());
			}, this.subsystems.coralIntake));

			Controller.rotateToFaceLeft.whileTrue(this.subsystems.swerve.new RotateToFaceCoralStation(RotateToFaceCoralStation.LEFT, true));
			Controller.rotateToFaceRight.whileTrue(this.subsystems.swerve.new RotateToFaceCoralStation(RotateToFaceCoralStation.RIGHT, true));

			this.controller.switches.b().onTrue(this.subsystems.hang.zeroHang());
			

			this.controller.switches.x().onTrue(new RunCommand(() -> {
				this.subsystems.hang.setFunnelRelease();
			}));

			this.controller.switches.a().onTrue(new InstantCommand(()
			-> {
				this.subsystems.elevator.resetPosition();
			}));

			this.controller.driverController.leftTrigger(0.2).whileTrue(this.subsystems.swerve.yawController(Controller.driveVector, () -> {return frc.robot.constants.Swerve.leftStationAngle;}, 1));
			this.controller.driverController.rightTrigger(0.2).whileTrue(this.subsystems.swerve.yawController(Controller.driveVector, () -> {return frc.robot.constants.Swerve.leftStationAngle;}, 1));
			this.controller.secondaryController.leftBumper().onTrue(this.bindings.bonk.up());
			this.controller.secondaryController.rightBumper().onTrue(this.bindings.bonk.down());
			this.controller.secondaryController.b().onTrue(this.bindings.bonk.middle());
			this.subsystems.hang.setDefaultCommand(
					new RunCommand(
							() -> {
								this.subsystems.hang
										.manualInput(this.controller.secondaryController.getRightTriggerAxis()
												- this.controller.secondaryController.getLeftTriggerAxis());
							}, this.subsystems.hang));
			this.controller.secondaryController.x().onTrue(new InstantCommand(()-> {
				Bonk.getInstance().setTarget(Targets.RESET);
			}, Bonk.getInstance()));
			//this.controller.secondaryController.a().onTrue(this.subsystems.bonk.bonk());

			// this.subsystems.algaeIntake.setDefaultCommand(this.subsystems.algaeIntake.manualInputCommand(this::getEject,
			// this::getAngle));

		}

		// Named commands
		{
			NamedCommands.registerCommand("align right", this.subsystems.swerve.new AlignToTagLeft(true));
			NamedCommands.registerCommand("align left", this.subsystems.swerve.new AlignToTagRight(true));
			NamedCommands.registerCommand("intake", this.bindings.coral.runIntake());
			NamedCommands.registerCommand("L0", this.bindings.elevator.goToL0());
			NamedCommands.registerCommand("compoundL2", this.bindings.coral.compoundL2());
			NamedCommands.registerCommand("compoundL4", this.bindings.coral.compoundL4());
			NamedCommands.registerCommand("L4", this.bindings.elevator.goToL4());
			NamedCommands.registerCommand("outtake", this.bindings.coral.outtake());
			NamedCommands.registerCommand("outtakeL4", this.bindings.coral.angleAndOuttakeL4());
			NamedCommands.registerCommand("gyro", Pgyro.setAutoGyro());
			NamedCommands.registerCommand("BonkUp", this.bindings.bonk.up());
			NamedCommands.registerCommand("BonkMiddle", this.bindings.bonk.middle());
			NamedCommands.registerCommand("BonkDown", this.bindings.bonk.down());
			NamedCommands.registerCommand("BonkReset", this.bindings.bonk.reset());
			NamedCommands.registerCommand("L2", this.bindings.elevator.goToL2());
			NamedCommands.registerCommand("print", new RunCommand(
				() -> {
					System.out.println("named command print");
				}, new Subsystem[]{}));
			NamedCommands.registerCommand("Wait Until In Funnel", this.subsystems.coralIntake.waitUntilCoralInFunnel());
			NamedCommands.registerCommand("Wait Until In Intake", new WaitUntilCommand(this.subsystems.coralIntake::legacyCoralDetected));
			NamedCommands.registerCommand("Intake Only", this.bindings.coral.runIntakeOnly());
			NamedCommands.registerCommand("Stop", this.bindings.swerve.stopDrivetrain());
			NamedCommands.registerCommand("Gyro + 90", Pgyro.setAutoGyroWithOffset(90));
			NamedCommands.registerCommand("Gyro - 90", Pgyro.setAutoGyroWithOffset(-90));
			NamedCommands.registerCommand("Init with vision", this.bindings.subsystems.swerve.resetPoseWithVision());
		
			
		}
		{
			autoChooser = AutoBuilder.buildAutoChooser("Score4L410L4Right"); //TODO: MAKE SURE THIS IS NOT SET DURING COMPETITION
			SmartDashboard.putData(autoChooser);
			//Shuffleboard.getTab("Autonomous").add("Auto Chooser", autoChooser);
		}
	}

	/**
	 * Returns the selected autonomous {@link Command}. Called in {@link Robot}.
	 * 
	 * @return The selected autonomous Command
	 */
	public Command getAutonomousCommand() {
		Command selectedAuto = autoChooser.getSelected();
		if (Objects.equals(selectedAuto, Commands.none())) {
			System.out.println("No Auto Selected");
			return Commands.none();
		} else {
			System.out.println("Auto Selected: " + selectedAuto);
			return selectedAuto;
		}
	}

	public void periodic() {
		Logger.updateEntries();
	}

	public double getAngle() {
		return (this.controller.driverController.getLeftTriggerAxis()
				- this.controller.driverController.getRightTriggerAxis()) / 5.0;
	}

	public double getEject() {
		if (this.controller.driverController.leftBumper().getAsBoolean()) {
			return 0.2;
		} else if (this.controller.driverController.rightBumper().getAsBoolean()) {
			return -0.2;
		} else {
			return 0;
		}

	}
}

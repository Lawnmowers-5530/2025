// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.containers.prod;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.PoseCameraManager;
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
		public PoseCameraManager man;
		public CoralIntake coralIntake;
		public Elevator elevator;
		public Hang hang;
	}

	public class State {}

	Controller controller = new Controller();

	private Subsystems subsystems;
	private Bindings bindings;

	public RobotContainer() {

		Logger.configureLoggingAndConfig(this, false);

		/**
		 * initalize subsystems here
		 */
		{

			this.subsystems = new Subsystems();

			this.subsystems.hang = new Hang();
			this.subsystems.man = new PoseCameraManager();
			this.subsystems.controller = new Controller();
			this.subsystems.coralIntake = new CoralIntake();
			this.subsystems.elevator = new Elevator();

			this.subsystems.swerve = new Swerve();
			// the death zone??
		}

		/**
		 * bind commands here
		 */
		{
			this.bindings = new Bindings(this.subsystems);

			this.subsystems.swerve.setDefaultCommand(this.subsystems.swerve.drive());

			Controller.L1.onTrue(this.bindings.elevator.goToL1());
			Controller.L2.onTrue(this.bindings.elevator.goToL2());
			Controller.L3.onTrue(this.bindings.elevator.goToL3());
			Controller.L4.onTrue(this.bindings.elevator.goToL4());

			Controller.zeroGyro.onTrue(this.bindings.swerve.zeroGyro());


			Controller.intake.onTrue(this.bindings.coral.runIntake());

			this.controller.secondaryController.x().onTrue(this.bindings.coral.manualElevator());
			this.controller.driverController.b().onTrue(this.bindings.coral.outtake());
			this.controller.driverController.a().whileTrue(
				new RunCommand(
					() -> {
						this.subsystems.coralIntake.manualPivot(this.controller.driverController.getLeftX());
					}, this.subsystems.coralIntake)
			);
			this.controller.driverController.povDown().onTrue(this.bindings.coral.outtake());

			this.subsystems.hang.setDefaultCommand(new RunCommand(()-> {
				this.subsystems.hang.manualInput(this.controller.secondaryController.getLeftY(), this.controller.secondaryController.getRightY());
			}, this.subsystems.hang));
			

			//this.subsystems.elevator.setDefaultCommand(this.bindings.elevator.manualElevator());

		}
	}

	/**
	 * Returns the selected autonomous {@link Command}. Called in {@link Robot}.
	 * 
	 * @return The selected autonomous Command
	 */
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public void periodic() {
		Logger.updateEntries();
	}
}

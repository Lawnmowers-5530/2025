// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.containers.elevator;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.containers.prod.RobotContainer.State.ControllerState;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.PoseCameraManager;
import frc.robot.subsystems.Elevator;
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

	private class Controllers {
		public CommandXboxController driverController;
		public CommandXboxController secondaryController;
	}

	class Subsystems {
		Swerve swerve;
		Controller controller;
		PoseCameraManager man;
		Elevator elevator;
	}

	public class Bindings {
		public Command swerveCommand;
		public Command zeroGyroCommand;
		public Command idTargeter;
		public Command align;

	}

	public static class State {
		public static class ControllerState {
			public static Vector<N2> driveVector;
			public static double driveRotation;
			public static boolean slowMode;
		}
	}

	public class Suppliers {
		public Supplier<Vector<N2>> driveVectorSupplier;
		public DoubleSupplier driveRotationSupplier;
	}

	private Controllers controllers;
	private Subsystems subsystems;
	private Bindings bindings;
	private Suppliers suppliers;

	public RobotContainer() {

		Logger.configureLoggingAndConfig(this, false);
		/**
		 * initalize controllers here
		 */
		{
			this.controllers = new Controllers();

			this.controllers.driverController = new CommandXboxController(0);
			this.controllers.secondaryController = new CommandXboxController(1);
		}

		/**
		 * initalize subsystems here
		 */
		{

			this.subsystems = new Subsystems();
			this.subsystems.elevator = new Elevator();
			this.subsystems.man = new PoseCameraManager();
			this.subsystems.controller = new Controller();

			this.subsystems.swerve = new Swerve();
			// the death zone??
		}

		/**
		 * initalize bindings here
		 */
		{

		}

		/** supps */
		{

		}

		this.bindings.swerveCommand = new RunCommand(
				() -> {
					this.subsystems.swerve.drive(
							ControllerState.driveVector.get(),
							ControllerState.driveRotation.get(),
							true,
							ControllerState.slowMode.get() ? 0.5 : 1);

				}, this.subsystems.swerve);

		// this.subsystems.swerve.setDefaultCommand(this.bindings.swerveCommand);
		this.subsystems.elevator.setDefaultCommand(new RunCommand(
				() -> {
					this.subsystems.elevator.setDirectSpeed(
							this.controllers.driverController.getLeftY() * 0.5);
				},
				this.subsystems.elevator));

		this.controllers.driverController.a().onTrue(this.subsystems.elevator.goToTarget(0));
		this.controllers.driverController.b().onTrue(this.subsystems.elevator.goToTarget(1));
		this.controllers.driverController.x().onTrue(this.subsystems.elevator.goToTarget(2));
		this.controllers.driverController.y().onTrue(this.subsystems.elevator.goToTarget(3));

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

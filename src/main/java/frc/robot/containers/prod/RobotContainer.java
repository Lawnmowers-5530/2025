// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.containers.prod;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.containers.prod.RobotContainer.State.ControllerState;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Pgyro;
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

	private class Controllers {
		public CommandXboxController driverController;
		public CommandXboxController secondaryController;
	}

	class Subsystems {
		Hang hang;
		Swerve swerve;
		Controller controller;
		PoseCameraManager man;
	}

	public class Bindings {
		public Command swerveCommand;
		public Command zeroGyroCommand;
		public Command idTargeter;
		public Command align;
		public Command releaseRatchetOnHang;
		public Command climbDeep;
		public Command toggleManual;
		public Command stopHang;
		public Command hang;
		public Command unhang;
		public Command stop;
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
			this.subsystems.hang = new Hang();
			this.subsystems.man = new PoseCameraManager();
			this.subsystems.controller = new Controller(this.controllers.driverController);

			this.subsystems.swerve = new Swerve();
			// the death zone??
		}

		/**
		 * initalize bindings here
		 */
		{
			this.bindings = new Bindings();

			// drive swerve, slow mode with b
			this.bindings.swerveCommand = this.subsystems.swerve.drive();

			// set gyro yaw to 0
			this.bindings.zeroGyroCommand = Pgyro.zeroGyroCommand();

			this.bindings.idTargeter = this.subsystems.swerve.getPointTargeterCommand(1, 0);
			this.subsystems.swerve.setDefaultCommand(this.bindings.swerveCommand);
			this.controllers.secondaryController.a().whileTrue(this.bindings.idTargeter);
			this.controllers.driverController.x().onTrue(this.bindings.zeroGyroCommand);

			this.bindings.align = this.subsystems.swerve.new AlignToTag(2);
			this.controllers.driverController.b().whileTrue(this.bindings.align);

			controllers.driverController.a().onTrue(this.bindings.climbDeep);
			controllers.driverController.b().onTrue(this.bindings.releaseRatchetOnHang);

		}

		/** supps */
		{
			this.suppliers = new Suppliers();
			this.suppliers.driveVectorSupplier = () -> {
				return VecBuilder.fill(this.controllers.driverController.getLeftX(),
						this.controllers.driverController.getLeftY());
			};
			this.suppliers.driveRotationSupplier = () -> {
				return this.controllers.driverController.getRightX();
			};
		}

		this.controllers.driverController.y().onTrue(this.bindings.hang);
		this.controllers.driverController.x().onTrue(this.bindings.unhang);
		this.controllers.driverController.a().onTrue(this.bindings.stop);

		this.controllers.driverController.leftStick().onFalse(this.bindings.stop);

		this.bindings.swerveCommand = new RunCommand(
				() -> {
					this.subsystems.swerve.drive(
							ControllerState.driveVector,
							ControllerState.driveRotation,
							true,
							ControllerState.slowMode ? 0.5 : 1);

				}, this.subsystems.swerve);
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

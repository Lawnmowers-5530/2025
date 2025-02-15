// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.containers.prod;


import java.util.function.Supplier;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.PoseCameraManager;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

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
	}

	public class State {
		public class ControllerState {
			@Log
			public static Supplier<Vector<N2>> driveVector;
			@Log
			public static Supplier<Double> driveRotation;
			@Log
			public static Supplier<Boolean> slowMode;
			@Log
			public static Trigger zeroGyro;
			
			public static Trigger L0;
			public static Trigger L1;
			public static Trigger L2;
			public static Trigger L3;
			public static Trigger L4;

			public static Trigger intake;
		}
	}

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

			State.ControllerState.L1.onTrue(this.bindings.elevator.goToL1);
			State.ControllerState.L2.onTrue(this.bindings.elevator.goToL2);
			State.ControllerState.L3.onTrue(this.bindings.elevator.goToL3);
			State.ControllerState.L4.onTrue(this.bindings.elevator.goToL4);

			State.ControllerState.zeroGyro.onTrue(this.bindings.swerve.zeroGyro);

			State.ControllerState.intake.onTrue(this.bindings.coral.runIntake);
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

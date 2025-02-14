// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Pgyro;
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
	DoubleLogEntry xLog;
	DoubleLogEntry yLog;
	DataLog log;
	private SendableChooser<Command> autoChooser;

	public class Subsystems {
		public Swerve swerve;
		public Controller controller;
		public PoseCameraManager man;
	}

	private class Bindings {
		public Command zeroGyroCommand;

	}

	public static class State {
		public static class ControllerState {
			@Log
			public static Supplier<Vector<N2>> driveVector;
			@Log
			public static Supplier<Double> driveRotation;
			@Log
			public static Supplier<Boolean> slowMode;
			@Log
			public static Trigger zeroGyro;
		}
	}

	Controller controller = new Controller();

	private Subsystems subsystems;
	private Bindings bindings;

	public RobotContainer() {
		DataLogManager.start();
		log = DataLogManager.getLog();
		xLog = new DoubleLogEntry(log, "/my/x");
		yLog = new DoubleLogEntry(log, "/my/y");

		Logger.configureLoggingAndConfig(this, false);

		/**
		 * initalize subsystems here
		 */
		{

			this.subsystems = new Subsystems();
			this.subsystems.man = new PoseCameraManager();

			this.subsystems.swerve = new Swerve();
			// the death zone??
		}

		/**
		 * initalize bindings here
		 */
		{
			this.bindings = new Bindings();

			// set gyro yaw to 0
			this.bindings.zeroGyroCommand = Pgyro.zeroGyroCommand();
		}

		/**
		 * bind commands here
		 */
		{
			this.subsystems.swerve.setDefaultCommand(this.subsystems.swerve.drive());

			State.ControllerState.zeroGyro.onTrue(this.bindings.zeroGyroCommand);
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
		xLog.append(this.subsystems.man.getEstimatedPoses().get(0).getFirst().estimatedPose.getTranslation().getX());
		yLog.append(this.subsystems.man.getEstimatedPoses().get(0).getFirst().estimatedPose.getTranslation().getY());
	}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.containers.hang;

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

	class Subsystems {
		Hang hang;
	}

	public class Bindings {

	}

	public class Controllers {
		public CommandXboxController controller1;
	}

	private Subsystems subsystems;
	private Controllers controllers;

	public RobotContainer() {

		Logger.configureLoggingAndConfig(this, false);

		/**
		 * initalize subsystems here
		 */
		{
			this.subsystems = new Subsystems();
			this.subsystems.hang = new Hang();
		}

		{
			this.controllers = new Controllers();
			this.controllers.controller1 = new CommandXboxController(0);
			this.subsystems.hang.setDefaultCommand(new RunCommand(() -> {
				this.subsystems.hang.setPower(-this.controllers.controller1.getRightY());
			}, this.subsystems.hang));
			this.controllers.controller1.a().whileTrue(new RunCommand(() -> {
				this.subsystems.hang.setReleased(true);
			}, this.subsystems.hang));
			this.controllers.controller1.b().whileTrue(new RunCommand(() -> {
				this.subsystems.hang.setReleased(false);
			}, this.subsystems.hang));
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

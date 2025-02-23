// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.containers.prod;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;
import frc.robot.subsystems.AlgaeIntake;
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


	
		


		//Watch for this
		public AlgaeIntake algaeIntake;
	
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


			//if (this.subsystems.algaeIntake == null) {
				//throw new IllegalStateException("This Code is Not Commented. Algae Intake is going to be Initialized");
			//}
			//this.subsystems.algaeIntake = new AlgaeIntake();
			

			autoChooser = AutoBuilder.buildAutoChooser();
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
			Controller.outtake.onTrue(this.bindings.coral.outtake());

			Controller.enableManualControl.whileTrue(this.bindings.coral.manualElevator().alongWith(this.bindings.coral.manualPivot()));

			Controller.alignLeft.whileTrue(this.subsystems.swerve.new AlignToTag(true));
			Controller.alignRight.whileTrue(this.subsystems.swerve.new AlignToTag(false));

			Controller.toggleLaserCan.onChange(new RunCommand(()-> {
				this.subsystems.coralIntake.setLaserCanSwitch(Controller.toggleLaserCan.getAsBoolean());
			}, this.subsystems.coralIntake));
			this.controller.switches.x().onChange(this.subsystems.hang.toggleFunnel());
			this.controller.driverController.start().onTrue(this.subsystems.hang.toggleFunnel());
		
			this.controller.driverController.povLeft().onTrue(this.bindings.coral.compoundL2());

			this.subsystems.hang.setDefaultCommand(
				new RunCommand(
					() -> {
						this.subsystems.hang.manualInput(this.controller.secondaryController.getRightTriggerAxis()-this.controller.secondaryController.getLeftTriggerAxis());
					}
					, this.subsystems.hang)
			);

			//this.subsystems.algaeIntake.setDefaultCommand(this.subsystems.algaeIntake.manualInputCommand(this::getEject, this::getAngle));

			

		}

		//Named commands
		{
			NamedCommands.registerCommand("align right", this.subsystems.swerve.new AlignToTag(true));
			NamedCommands.registerCommand("intake", this.bindings.coral.runIntake());
			NamedCommands.registerCommand("L0", this.bindings.elevator.goToL0());
			NamedCommands.registerCommand("compoundL2", this.bindings.coral.compoundL2());
			NamedCommands.registerCommand("compoundL4", this.bindings.coral.compoundL4());
		}
	}

	/**
	 * Returns the selected autonomous {@link Command}. Called in {@link Robot}.
	 * 
	 * @return The selected autonomous Command
	 */
	public Command getAutonomousCommand() {
		return AutoBuilder.buildAuto("autotest");
	}

	public void periodic() {
		Logger.updateEntries();
	}

	public double getAngle() {
		return (this.controller.driverController.getLeftTriggerAxis() - this.controller.driverController.getRightTriggerAxis()) / 5.0;
	}
	public double getEject() {
		if (this.controller.driverController.leftBumper().getAsBoolean()) {
			return 0.2;
		}else if (this.controller.driverController.rightBumper().getAsBoolean()) {
			return -0.2;
		}else {
			return 0;
		}

	}
}

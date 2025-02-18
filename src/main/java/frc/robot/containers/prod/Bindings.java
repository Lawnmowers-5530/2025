package frc.robot.containers.prod;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.CoralIntake.Targets;
import frc.robot.subsystems.Pgyro;

public class Bindings {
	RobotContainer.Subsystems subsystems;

	public Bindings(RobotContainer.Subsystems subsystems) {
		this.subsystems = subsystems;
		swerve = this.new Swerve();
		intake = this.new Intake();
		elevator = this.new Elevator();
		coral = this.new Coral();
		

	}

	Elevator elevator;

	Intake intake;

	final class Intake {
		/**
		 * Angle pivot to intake angle / L2 & L3 angle
		 */
		public Command angleIntake = Bindings.this.subsystems.coralIntake.anglePivot(Targets.INTAKE);
	}

	final class Elevator {
		/**
		 * Intake angle, then move elevator to L0, ends when within tolerance of target
		 */
		final Command goToL0 = Bindings.this.intake.angleIntake
				.andThen(Bindings.this.subsystems.elevator.goToTarget(0))
				.until(Bindings.this.subsystems.elevator::atTarget);
		/**
		 * Intake angle, then move elevator to L1, ends when within tolerance of target
		 */
		final Command goToL1 = Bindings.this.subsystems.elevator.goToTarget(1)
				.until(Bindings.this.subsystems.elevator::atTarget);
		/**
		 * Intake angle, then move elevator to L2, ends when within tolerance of target
		 */
		final Command goToL2 = Bindings.this.intake.angleIntake
				.andThen(Bindings.this.subsystems.elevator.goToTarget(2))
				.until(Bindings.this.subsystems.elevator::atTarget);
		
		/**
		 * Intake angle, then move elevator to L3, ends when within tolerance of target
		 */
		final Command goToL3 = Bindings.this.intake.angleIntake
				.andThen(Bindings.this.subsystems.elevator.goToTarget(3))
				.until(Bindings.this.subsystems.elevator::atTarget);
		/**
		 * Intake angle, then move elevator to L4, ends when within tolerance of target.
		 * Then angles to L4
		 */
		final Command goToL4 = Bindings.this.intake.angleIntake
				.andThen(Bindings.this.subsystems.elevator.goToTarget(4))
				.until(Bindings.this.subsystems.elevator::atTarget)
				.andThen(Bindings.this.subsystems.coralIntake.anglePivot(Targets.TOP));
	}

	Swerve swerve;

	final class Swerve {
		final Command zeroGyro = Pgyro.zeroGyroCommand();
	}

	Coral coral;

	final class Coral {
		/**
		 * Move elevator to L0, angle to intake, run intake, and stop intake when coral
		 * detected
		 */
		public Command runIntake = (Bindings.this.elevator.goToL0
				.alongWith(Bindings.this.subsystems.coralIntake.anglePivot(Targets.INTAKE)))
				.until(Bindings.this.subsystems.elevator::atTarget)
				.andThen(Bindings.this.subsystems.coralIntake.intakeCommand())
				.until(Bindings.this.subsystems.coralIntake::coralDetected1)
				.andThen(Bindings.this.subsystems.coralIntake.stopIntakeCommand());

		public Command outtake = Bindings.this.subsystems.coralIntake.intakeCommand()
				.until(Bindings.this.subsystems.coralIntake::notCoralDetected1)
				.andThen(Bindings.this.subsystems.coralIntake.stopIntakeCommand());
		/**
		 * Move elevator and angle and outtake coral at L2, built for auton use
		 */
		public Command compoundL2 = Bindings.this.subsystems.coralIntake.anglePivot(Targets.TOP)
				.andThen(Bindings.this.elevator.goToL2)
				.andThen(Bindings.this.subsystems.coralIntake.anglePivot(Targets.MIDDLE))
				.andThen(Bindings.this.subsystems.coralIntake.intakeCommand())
				.until(Bindings.this.subsystems.coralIntake::notCoralDetected1)
				.andThen(Bindings.this.subsystems.coralIntake.stopIntakeCommand());
		/**
		 * Move elevator and angle and outtake coral at L3, built for auton use
		 */
		public Command compoundL3 = Bindings.this.subsystems.coralIntake.anglePivot(Targets.TOP)
				.andThen(Bindings.this.elevator.goToL3)
				.andThen(Bindings.this.subsystems.coralIntake.anglePivot(Targets.MIDDLE))
				.andThen(Bindings.this.subsystems.coralIntake.intakeCommand())
				.until(Bindings.this.subsystems.coralIntake::notCoralDetected1)
				.andThen(Bindings.this.subsystems.coralIntake.stopIntakeCommand());
		/**
		 * Move elevator and angle and outtake coral at L4, built for auton use
		 */
		public Command compoundL4 = Bindings.this.subsystems.coralIntake.anglePivot(Targets.TOP)
				.andThen(Bindings.this.elevator.goToL4)
				.andThen(Bindings.this.subsystems.coralIntake.intakeCommand())
				.until(Bindings.this.subsystems.coralIntake::notCoralDetected1)
				.andThen(Bindings.this.subsystems.coralIntake.stopIntakeCommand());
		/**
		 * Angle pivot to L4 angle
		 */
		public Command angleL4 = Bindings.this.subsystems.coralIntake.anglePivot(Targets.TOP);

		/**
		 * Manually manipulate {@link frc.robot.subsystems.Elevator Elevator} with
		 * {@link frc.robot.subsystems.Controller Controller} Elevatorpower value
		 */
		final Command manualElevator = new RunCommand(
				() -> {
					Bindings.this.subsystems.elevator
							.manualSetSpeed(subsystems.controller.elevatorPower());
				}, Bindings.this.subsystems.elevator);

	}

	final class Hang {
	}
}

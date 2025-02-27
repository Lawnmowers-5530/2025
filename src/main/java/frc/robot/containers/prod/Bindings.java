package frc.robot.containers.prod;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Controller;
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

	final Command printSomething(String something) {
		return new InstantCommand(
				() -> {
					System.out.println(something);
				});

	}

	Elevator elevator;

	Intake intake;

	final class Intake {
		/**
		 * Angle pivot to intake angle / L2 & L3 angle
		 */
		Command angleIntake() {
			return Bindings.this.subsystems.coralIntake.anglePivot(Targets.INTAKE);
		}
	}

	final class Elevator {

		/**
		 * Intake angle, then move elevator to L0, ends when within tolerance of target
		 */
		Command goToL0() {
			return (Bindings.this.subsystems.coralIntake.anglePivot(Targets.TOP)
					.alongWith(Bindings.this.subsystems.elevator.goToTarget(0)))
					.until(Bindings.this.subsystems.elevator::atTarget)
					.andThen(Bindings.this.subsystems.coralIntake.anglePivot(Targets.INTAKE));
		}

		/**
		 * Intake angle, then move elevator to L1, ends when within tolerance of target
		 */
		Command goToL1() {
			return (Bindings.this.subsystems.coralIntake.anglePivot(Targets.TOP)
					.alongWith(Bindings.this.subsystems.elevator.goToTarget(1)))
					.until(Bindings.this.subsystems.elevator::atTarget)
					.andThen(Bindings.this.subsystems.coralIntake.anglePivot(Targets.BOTTOM));
		}

		/**
		 * Intake angle, then move elevator to L2, ends when within tolerance of target
		 */
		Command goToL2() {
			return (Bindings.this.subsystems.coralIntake.anglePivot(Targets.TOP)
					.alongWith(Bindings.this.subsystems.elevator.goToTarget(2)))
					.until(Bindings.this.subsystems.elevator::atTarget)
					.andThen(Bindings.this.subsystems.coralIntake.anglePivot(Targets.MIDDLE));
		}

		/**
		 * Intake angle, then move elevator to L3, ends when within tolerance of target
		 */
		Command goToL3() {
			return (Bindings.this.subsystems.coralIntake.anglePivot(Targets.TOP)
					.alongWith(Bindings.this.subsystems.elevator.goToTarget(3)))
					.until(Bindings.this.subsystems.elevator::atTarget)
					.andThen(Bindings.this.subsystems.coralIntake.anglePivot(Targets.MIDDLE));
		}

		/**
		 * Intake angle, then move elevator to L4, ends when within tolerance of target.
		 * Then angles to L4
		 */
		Command goToL4() {
			return ((Bindings.this.subsystems.coralIntake.anglePivot(Targets.TOP)
					.alongWith(Bindings.this.subsystems.elevator.goToTarget(4)))
					.until(Bindings.this.subsystems.elevator::atTarget)
					.andThen(Bindings.this.subsystems.coralIntake.anglePivot(Targets.L4)))
					.unless(Bindings.this.subsystems.elevator::tooHigh);
		}
	}

	Swerve swerve;

	final class Swerve {
		Command zeroGyro() {
			return Pgyro.zeroGyroCommand();
		}

		final class SlowModeCommand extends Command {
			@Override
			public void initialize() {
				subsystems.swerve.setSlowMode(true);
			}

			@Override
			public void end(boolean _interrupted) {
				subsystems.swerve.setSlowMode(false);
			}
		}

		Command slowMode() {
			return new SlowModeCommand();
		} // use whileTrue on trigger only, command will not end properly when using
			// onTrue on trigger
	}

	Coral coral;

	final class Coral {
		public boolean pivotAndElevator() {
			return Bindings.this.subsystems.coralIntake.atTarget.getAsBoolean()
					&& Bindings.this.subsystems.elevator.atTarget();
		}

		/**
		 * Move elevator to L0, angle to intake, run intake, and stop intake when coral
		 * detected
		 */
		Command runIntake() {
			return Bindings.this.elevator.goToL0()
					.until(Bindings.this.subsystems.elevator::atTarget)
					.andThen(Bindings.this.subsystems.coralIntake.intakeCommand())
					.until(Bindings.this.subsystems.coralIntake::coralDetected)
					.andThen(Bindings.this.subsystems.coralIntake.stopIntakeCommand());
		}

		Command outtake() {
			return Bindings.this.subsystems.coralIntake.intakeCommand()
					.until(Bindings.this.subsystems.coralIntake::notCoralDetected)
					.andThen(Bindings.this.subsystems.coralIntake.stopIntakeCommand());
		}

		Command angleAndOuttakeL4() {
			return Bindings.this.subsystems.coralIntake.anglePivot(Targets.L4)
					.andThen(new WaitUntilCommand(this::pivotAndElevator))
					.andThen(Bindings.this.coral.outtake());
		}

		/**
		 * Move elevator and angle and outtake coral at L2, built for auton use
		 */
		Command compoundL2() {
			return Bindings.this.elevator.goToL2()
					.andThen(new WaitUntilCommand(this::pivotAndElevator))
					.andThen(Bindings.this.coral.outtake());
		}

		/**
		 * Move elevator and angle and outtake coral at L3, built for auton use
		 */
		Command compoundL3() {
			return Bindings.this.elevator.goToL3()
					.andThen(new WaitUntilCommand(this::pivotAndElevator))
					.andThen(Bindings.this.coral.outtake());
		}

		/**
		 * Move elevator and angle and outtake coral at L4, built for auton use
		 */
		Command compoundL4() {
			return Bindings.this.elevator.goToL4()
					.andThen(new WaitUntilCommand(this::pivotAndElevator))
					.andThen(Bindings.this.coral.outtake())
					.until(Bindings.this.subsystems.coralIntake::notCoralDetected);
		}

		/**
		 * Angle pivot to L4 angle
		 */
		Command angleL4() {
			return Bindings.this.subsystems.coralIntake.anglePivot(Targets.TOP);
		}

		/**
		 * Manually manipulate {@link frc.robot.subsystems.Elevator Elevator} with
		 * {@link frc.robot.subsystems.Controller Controller} Elevatorpower value
		 */
		Command manualElevator() {
			return new RunCommand(
					() -> {
						Bindings.this.subsystems.elevator
								.manualSetSpeed(Controller.manualElevatorPower.get());
					}, Bindings.this.subsystems.elevator);
		}

		Command manualPivot() {
			return new RunCommand(
					() -> {
						Bindings.this.subsystems.coralIntake
								.manualPivot(Controller.manualPivotPower.get());
					}, Bindings.this.subsystems.coralIntake);
		}

	}

	final class Hang {
	}
}

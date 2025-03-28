package frc.robot.containers.prod;

import javax.naming.Binding;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Bonk;
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
		bonk = this.new Bonker();



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

	Bonker bonk;
	public final class Bonker {
		Command up() {
			return new InstantCommand(()-> {
				Bonk.getInstance().setTarget(frc.robot.subsystems.Bonk.Targets.UP);
			}, Bonk.getInstance());
		}
		Command down() {
			return new InstantCommand(()-> {
				Bonk.getInstance().setTarget(frc.robot.subsystems.Bonk.Targets.DOWN);
			}, Bonk.getInstance());
		
		}
		Command middle() {
			return new InstantCommand(()-> {
				Bonk.getInstance().setTarget(frc.robot.subsystems.Bonk.Targets.MIDDLE);
			}, subsystems.bonk);
		}
		Command reset() {
			return new InstantCommand(()-> {
				Bonk.getInstance().setTarget(frc.robot.subsystems.Bonk.Targets.RESET);
			}, subsystems.bonk);
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
		Command stopDrivetrain() {
			return new InstantCommand(()-> {
				Bindings.this.subsystems.swerve.autoDriveRobotRelative(new ChassisSpeeds(), 0);
			}, Bindings.this.subsystems.swerve);
		}
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
					.andThen(new WaitCommand(0.15))
					.andThen(Bindings.this.subsystems.coralIntake.stopIntakeCommand());
		}
		Command runIntakeOnly() {
			return 
					Bindings.this.subsystems.coralIntake.intakeCommand()
					.until(Bindings.this.subsystems.coralIntake::coralDetected)
					.andThen(new WaitCommand(0.15))
					.andThen(Bindings.this.subsystems.coralIntake.stopIntakeCommand());
		}

		Command outtake() {
			return Bindings.this.subsystems.coralIntake.intakeCommand()
					.until(Bindings.this.subsystems.coralIntake::notCoralDetected)
					.andThen(new WaitCommand(0.2))
					.andThen(Bindings.this.subsystems.coralIntake.stopIntakeCommand());
		}
		Command conditionalOuttake() {
			return new RunCommand(()-> {
				if (Bindings.this.subsystems.elevator.sp == frc.robot.constants.Elevator.level1) {
					Bindings.this.subsystems.coralIntake.outtakeL1();
				}else {
					Bindings.this.subsystems.coralIntake.intake();
				}
			}, Bindings.this.subsystems.coralIntake).until(Bindings.this.subsystems.coralIntake::notCoralDetected)
			.andThen(new WaitCommand(0.2))
			.andThen(Bindings.this.subsystems.coralIntake.stopIntakeCommand());

		}
		Command outtakeL1() {
			return Bindings.this.subsystems.coralIntake.outtakeL1Command()
					.until(Bindings.this.subsystems.coralIntake::notCoralDetected)
					.andThen(Bindings.this.subsystems.coralIntake.stopIntakeCommand());
		}

		Command angleAndOuttakeL4() {
			return Bindings.this.subsystems.coralIntake.anglePivot(Targets.L4)
					.andThen(new WaitUntilCommand(this::pivotAndElevator))
					.andThen(Bindings.this.coral.outtake());
		}

		Command compoundL1() {
			return Bindings.this.elevator.goToL1()
					.andThen(new WaitUntilCommand(this::pivotAndElevator))
					.andThen(Bindings.this.coral.outtakeL1());
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

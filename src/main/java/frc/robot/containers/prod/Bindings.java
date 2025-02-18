package frc.robot.containers.prod;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.CoralIntake.Targets;
import frc.robot.containers.prod.RobotContainer.Subsystems;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Pgyro;

public class Bindings {
    RobotContainer.Subsystems subsystems;

    public Bindings(RobotContainer.Subsystems subsystems) {
        this.subsystems = subsystems;
        elevator = this.new Elevator();
        swerve = this.new Swerve();
        coral = this.new Coral();

    }

    Elevator elevator;
    final class Elevator {
        final Command manualElevator = new RunCommand(
                () -> {
                    Bindings.this.subsystems.elevator.manualSetSpeed(subsystems.controller.elevatorPower());
                }, Bindings.this.subsystems.elevator);

        final Command goToL0 = Bindings.this.subsystems.elevator.goToTarget(0);
        final Command goToL1 = Bindings.this.subsystems.elevator.goToTarget(1);
        final Command goToL2 = Bindings.this.subsystems.elevator.goToTarget(2);// TODO add angle to L4 while in transit
        final Command goToL3 = Bindings.this.subsystems.elevator.goToTarget(3);
        final Command goToL4 = Bindings.this.subsystems.elevator.goToTarget(4);
    }

    Swerve swerve;

    final class Swerve {
        final Command zeroGyro = Pgyro.zeroGyroCommand();
    }

    Coral coral;
    final class Coral {
        public Command runIntake = Bindings.this.subsystems.elevator.goToTarget(0)
                .until(Bindings.this.subsystems.elevator::atTarget)
                .andThen(Bindings.this.subsystems.coralIntake.intakeCommand())
                .until(Bindings.this.subsystems.coralIntake::coralDetected1)
                .andThen(Bindings.this.subsystems.coralIntake.stopIntakeCommand());
        public Command scoreL4 = Bindings.this.subsystems.coralIntake.anglePivot(Targets.TOP);
        public Command outtake = new RunCommand(() -> {
            Bindings.this.subsystems.coralIntake.outtake();
        }, Bindings.this.subsystems.coralIntake).until(Bindings.this.subsystems.coralIntake::notCoralDetected1)
        .andThen(Bindings.this.subsystems.coralIntake.stopIntakeCommand());

    }

    final class Hang {
    }
}

package frc.robot.containers.prod;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.CoralIntake.Targets;
import frc.robot.subsystems.Pgyro;

public class Bindings {
    RobotContainer.Subsystems subsystems;

    public Bindings(RobotContainer.Subsystems subsystems) {
        this.subsystems = subsystems;
    }

    Elevator elevator = this.new Elevator();

    final class Elevator {
        final Command goToL0 = Bindings.this.subsystems.elevator.goToTarget(0);
        final Command goToL1 = Bindings.this.subsystems.elevator.goToTarget(1);
        final Command goToL2 = Bindings.this.subsystems.elevator.goToTarget(2);//TODO add angle to L4 while in transit
        final Command goToL3 = Bindings.this.subsystems.elevator.goToTarget(3);
        final Command goToL4 = Bindings.this.subsystems.elevator.goToTarget(4);
    }

    Swerve swerve = this.new Swerve();

    final class Swerve {
        final Command zeroGyro = Pgyro.zeroGyroCommand();
    }

    Coral coral = this.new Coral();
    final class Coral {
        public Command runIntake = Bindings.this.subsystems.elevator.goToTarget(0)
                .until(Bindings.this.subsystems.elevator::atTarget)
                .andThen(Bindings.this.subsystems.coralIntake.intakeCommand())
                .until(Bindings.this.subsystems.coralIntake::coralDetected1)
                .andThen(Bindings.this.subsystems.coralIntake.stopIntakeCommand());
        public Command scoreL4 = Bindings.this.subsystems.coralIntake.anglePivot(Targets.TOP);
    }
}

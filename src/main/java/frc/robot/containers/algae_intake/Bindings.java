package frc.robot.containers.algae_intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class Bindings {
    RobotContainer.Subsystems subsystems;

    public Bindings(RobotContainer.Subsystems subsystems) {
        this.subsystems = subsystems;
        elevator = this.new AlgaeIntake();
    }

    AlgaeIntake elevator;
    public final class AlgaeIntake {
        public final Command manualControl = new RunCommand(() -> subsystems.algaeIntake.setManualControl(subsystems.controller.driverController.getRightTriggerAxis()));
        public final Command stop = new RunCommand(() -> subsystems.algaeIntake.stop());
        public final Command intake = new RunCommand(() -> subsystems.algaeIntake.intake());
        public final Command eject = new RunCommand(() -> subsystems.algaeIntake.eject());
    }
}

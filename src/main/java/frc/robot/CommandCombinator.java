package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer.Subsystems;

/**
 * Class for combining commands defined in {@link RobotContainer}. Efficent way
 * to make different sequences of commands.
 */
public class CommandCombinator {
//	Subsystems subsystems;
//
//	public CommandCombinator(RobotContainer.Subsystems subsystems) {
//		this.subsystems = subsystems;
//	}
//
//	public Command exampleCombination(double angle) {
//		return new SequentialCommandGroup(
//				this.subsystems.swerve.yawController(
//						() -> {
//							return VecBuilder.fill(0, 0);
//						}, () -> {
//							return angle;
//						},
//						1),
//				logFinish("exampleCombination"));
//	}
//
//	private Command logFinish(String cmdName) {
//		return new InstantCommand(
//				() -> {
//					System.out.println(cmdName + " finished");
//				},
//				new Subsystem[] {});
//	}
}

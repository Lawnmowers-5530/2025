package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.Container;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The {@link Controller} class is a subsystem that is responsible for handling
 * all controller input and updating the {@link Controller} accordingly.
 * Done here so we only do the math once, and every subsystem using the same
 * data
 */
public class Controller extends SubsystemBase {
	static final class ControllerConstants extends frc.robot.constants.Controller {
	};

	public static Supplier<Vector<N2>> driveVector;
	public static Supplier<Double> driveRotation;
	public static Supplier<Double> manualElevatorPower;
	public static Supplier<Double> manualPivotPower;
	public static Trigger zeroGyro;
	public static Trigger L1;
	public static Trigger L2;
	public static Trigger L3;
	public static Trigger L4;
	public static Trigger angleL4;
	public static Trigger enableManualControl;
	public static Trigger outtake;
	public static Trigger intake;
	public static Trigger funnel;
	public static Trigger alignLeft;
	public static Trigger alignRight;
	public static Trigger toggleLaserCan;


	public static Trigger slowMode;

	public CommandXboxController driverController;
	public CommandXboxController secondaryController;
	public CommandXboxController switches;

	public Controller() {
		this.driverController = new CommandXboxController(0);
		this.secondaryController = new CommandXboxController(1);
		this.switches = new CommandXboxController(2);

		// driver controller
		{
			driveVector = () -> {
				return VecBuilder.fill(
						MathUtil.applyDeadband(
								-driverController.getLeftY(),
								ControllerConstants.driveControllerJoystickDeadband,
								1),
						MathUtil.applyDeadband(
								-driverController.getLeftX(),
								ControllerConstants.driveControllerJoystickDeadband,
								1));
			};

			driveRotation = () -> {
				return MathUtil.applyDeadband(
						-driverController.getRightX(),
						ControllerConstants.driveControllerJoystickDeadband,
						1);
			};
			slowMode = driverController.b();
			zeroGyro = driverController.x();
			intake = driverController.y();
			outtake = driverController.a();
			alignLeft = driverController.povLeft();
			alignRight = driverController.povRight();
		}

		// secondary controller
		{
			funnel = switches.x(); // TODO

			L1 = secondaryController.povDown();
			L2 = secondaryController.povLeft();
			L3 = secondaryController.povUp();
			L4 = secondaryController.povRight();

			enableManualControl = new Trigger(() -> {
				return Math.abs(secondaryController.getLeftY()) > 0.02
						|| Math.abs(secondaryController.getRightY()) > 0.02;
			});

			manualElevatorPower = () -> {
				return -this.secondaryController.getLeftY();
			};

			manualPivotPower = () -> {
				return -this.secondaryController.getRightY();
			};

			toggleLaserCan = this.switches.y();
		}
	}
}
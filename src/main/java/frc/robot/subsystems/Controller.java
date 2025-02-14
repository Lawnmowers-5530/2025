package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot.Container;


/**
 * The {@link Controller} class is a subsystem that is responsible for handling
 * all controller input and updating the {@link Controller} accordingly.
 * Done here so we only do the math once, and every subsystem using the same data
 */
public class Controller extends SubsystemBase {
    static final class ControllerConstants extends frc.robot.constants.Controller {};

    public CommandXboxController driverController;
    public CommandXboxController secondaryController;

    public Controller() {
        this.driverController = new CommandXboxController(0);
        this.secondaryController = new CommandXboxController(1);

        Container.State.ControllerState.driveVector = () -> {
            return VecBuilder.fill(
                MathUtil.applyDeadband(
                    -driverController.getLeftY(),
                    ControllerConstants.driveControllerJoystickDeadband,
                    1),
                MathUtil.applyDeadband(
                    -driverController.getLeftX(),
                    ControllerConstants.driveControllerJoystickDeadband,
                    1)
            );
        };

        ControllerState.driveRotation = () -> {
            return MathUtil.applyDeadband(
                -driverController.getRightX(),
                ControllerConstants.driveControllerJoystickDeadband,
                1
            );
        };

        ControllerState.slowMode = () -> {
            return driverController.b().getAsBoolean();
        };

        ControllerState.zeroGyro = driverController.x();
    }

    //public void periodic() {
//
    //    ControllerState.driveVector = VecBuilder.fill(
	//		MathUtil.applyDeadband(
	//			driverController.getLeftY(),
	//			ControllerConstants.driveControllerJoystickDeadband,
	//			1),
	//		MathUtil.applyDeadband(
	//			-driverController.getLeftX(),
	//			ControllerConstants.driveControllerJoystickDeadband,
	//			1)
    //    );
//
    //    ControllerState.driveRotation = MathUtil.applyDeadband(
    //        -driverController.getRightX(),
    //        ControllerConstants.driveControllerJoystickDeadband,
    //        1
    //    );
//
    //    ControllerState.slowMode = driverController.b().getAsBoolean();
    //}
}
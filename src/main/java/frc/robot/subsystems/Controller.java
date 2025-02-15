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

    private CommandXboxController driverController;
    private CommandXboxController secondaryController;

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

        Container.State.ControllerState.driveRotation = () -> {
            return MathUtil.applyDeadband(
                -driverController.getRightX(),
                ControllerConstants.driveControllerJoystickDeadband,
                1
            );
        };

        Container.State.ControllerState.slowMode = () -> {
            return driverController.b().getAsBoolean();
        };

        Container.State.ControllerState.zeroGyro = driverController.x();

        Container.State.ControllerState.L0 = driverController.a();
        Container.State.ControllerState.L1 = driverController.povDown();
        Container.State.ControllerState.L2 = driverController.povLeft();
        Container.State.ControllerState.L3 = driverController.povUp();
        Container.State.ControllerState.L4 = driverController.povRight();

        Container.State.ControllerState.intake = driverController.y();
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
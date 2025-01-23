package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.RobotContainer.State.ControllerState;


/**
 * The {@link Controller} class is a subsystem that is responsible for handling
 * all controller input and updating the {@link Controller} accordingly.
 * Done here so we only do the math once, and every subsystem using the same data
 */
public class Controller extends SubsystemBase {
    public CommandXboxController driverController;

    public Controller(CommandXboxController driverController) {
        this.driverController = driverController;
    }

    public void periodic() {

        ControllerState.driveVector = VecBuilder.fill(
			MathUtil.applyDeadband(
				driverController.getLeftY(),
				ControllerConstants.driveControllerJoystickDeadband,
				1),
			MathUtil.applyDeadband(
				-driverController.getLeftX(),
				ControllerConstants.driveControllerJoystickDeadband,
				1)
        );

        ControllerState.driveRotation = MathUtil.applyDeadband(
            -driverController.getRightX(),
            ControllerConstants.driveControllerJoystickDeadband,
            1
        );

        ControllerState.slowMode = driverController.b().getAsBoolean();
    }
}
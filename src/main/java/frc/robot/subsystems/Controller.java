package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    public CommandXboxController driverController;
    public CommandXboxController secondaryController;

    public Controller() {
        this.driverController = new CommandXboxController(0);
        this.secondaryController = new CommandXboxController(1);

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

        slowMode = () -> {
            return driverController.b().getAsBoolean();
        };

        zeroGyro = driverController.x();

        L1 = driverController.povDown();
        L2 = driverController.povLeft();
        L3 = driverController.povUp();
        L4 = driverController.povRight();

        intake = driverController.y();
    }

    public static Supplier<Vector<N2>> driveVector;
    public static Supplier<Double> driveRotation;
    public static Supplier<Boolean> slowMode;
    public static Trigger zeroGyro;
    public static Trigger L1;
    public static Trigger L2;
    public static Trigger L3;
    public static Trigger L4;
    public static Trigger angleL4;

    public static Trigger intake;

    public double elevatorPower() {
        return this.secondaryController.getLeftX();
    }

    // public void periodic() {
    //
    // ControllerState.driveVector = VecBuilder.fill(
    // MathUtil.applyDeadband(
    // driverController.getLeftY(),
    // ControllerConstants.driveControllerJoystickDeadband,
    // 1),
    // MathUtil.applyDeadband(
    // -driverController.getLeftX(),
    // ControllerConstants.driveControllerJoystickDeadband,
    // 1)
    // );
    //
    // ControllerState.driveRotation = MathUtil.applyDeadband(
    // -driverController.getRightX(),
    // ControllerConstants.driveControllerJoystickDeadband,
    // 1
    // );
    //
    // ControllerState.slowMode = driverController.b().getAsBoolean();
    // }
}
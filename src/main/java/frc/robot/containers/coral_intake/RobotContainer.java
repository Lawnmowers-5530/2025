// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.containers.coral_intake;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Pgyro;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.PoseCameraManager;
import io.github.oblarg.oblog.Logger;
import frc.robot.Robot;

/**
 * The {@link RobotContainer} holds all subsystems, commands, suppliers, etc. in
 * a command-based structure. The only action in {@link Robot} is to schedule
 * the commands defined here, and to create an instance of the
 * {@link RobotContainer}, which will also bind all buttons to their respective
 * commands.
 */
public class RobotContainer {
    private SendableChooser<Command> autoChooser;

    private class Controllers {
        public CommandXboxController driverController;
        public CommandXboxController secondaryController;
    }

    class Subsystems {
        Swerve swerve;
        Controller controller;
        PoseCameraManager man;
        CoralIntake coralIntake;
    }

    public class Bindings {
        public Command swerveCommand;
        public Command zeroGyroCommand;
        public Command idTargeter;
        public Command align;

    }

    public static class State {
        public static class ControllerState {
            public static Vector<N2> driveVector;
            public static double driveRotation;
            public static boolean slowMode;
        }
    }

    public class Suppliers {
        public Supplier<Vector<N2>> driveVectorSupplier;
        public DoubleSupplier driveRotationSupplier;
    }

    private Controllers controllers;
    private Subsystems subsystems;
    private Bindings bindings;
    private Suppliers suppliers;

    public RobotContainer() {
        CanBridge.runTCP();

        Logger.configureLoggingAndConfig(this, false);
        /**
         * initalize controllers here
         */
        {
            this.controllers = new Controllers();

            this.controllers.driverController = new CommandXboxController(0);
            this.controllers.secondaryController = new CommandXboxController(1);
        }

        /**
         * initalize subsystems here
         */
        {

            this.subsystems = new Subsystems();
            this.subsystems.man = new PoseCameraManager();
            this.subsystems.controller = new Controller(this.controllers.driverController);
            this.subsystems.coralIntake = new CoralIntake();

            // this.subsystems.swerve = new Swerve();
            // the death zone??
        }

        /**
         * initalize bindings here
         */
        {
            this.bindings = new Bindings();

            this.subsystems.coralIntake.setLaserCanSwitch(true);

            this.controllers.driverController.a().onTrue(new RunCommand(() -> {
                this.subsystems.coralIntake.setTarget(CoralIntake.Targets.INTAKE);
            }));
            this.controllers.driverController.b().onTrue(new RunCommand(() -> {
                this.subsystems.coralIntake.setTarget(CoralIntake.Targets.BOTTOM);
            }));
            this.controllers.driverController.x().onTrue(new RunCommand(() -> {
                this.subsystems.coralIntake.setTarget(CoralIntake.Targets.MIDDLE);
            }));
            this.controllers.driverController.y().onTrue(new RunCommand(() -> {
                this.subsystems.coralIntake.setTarget(CoralIntake.Targets.TOP);
            }));

            this.controllers.driverController.leftBumper().onTrue(new RunCommand(() -> {
                this.subsystems.coralIntake.intake();
            }).until(this.subsystems.coralIntake::coralDetected).andThen(this.subsystems.coralIntake::stopIntake));
            this.controllers.driverController.rightBumper().onTrue(new RunCommand(() -> {
                this.subsystems.coralIntake.outtake();
            }).until(this.subsystems.coralIntake::coralDetected).andThen(this.subsystems.coralIntake::stopIntake));
        }

        /**supps */
        {
            this.suppliers = new Suppliers();
            this.suppliers.driveVectorSupplier = () -> {
                return VecBuilder.fill(this.controllers.driverController.getLeftX(), this.controllers.driverController.getLeftY());
            };
            this.suppliers.driveRotationSupplier = () -> {
                return this.controllers.driverController.getRightX();
            };
        }
    }

    /**
     * Returns the selected autonomous {@link Command}. Called in {@link Robot}.
     *
     * @return The selected autonomous Command
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void periodic() {
        Logger.updateEntries();
    }
}
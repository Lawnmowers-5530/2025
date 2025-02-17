// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.containers.algae_intake;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.vision.PoseCameraManager;
import io.github.oblarg.oblog.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

    public class Subsystems {
        Swerve swerve;
        Controller controller;
        PoseCameraManager man;
        AlgaeIntake algaeIntake
    }

    private Controllers controllers;
    private Subsystems subsystems;
    private Bindings bindings;

    public RobotContainer() {

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
            this.subsystems.controller = new Controller();
            this.subsystems.algaeIntake = new AlgaeIntake();

            // this.subsystems.swerve = new Swerve();
            // the death zone??
        }

        /**
         * initalize bindings here
         */
        {
            this.bindings = new Bindings(this.subsystems);



        }

        /**supps */
        {
            this.suppliers = new Suppliers();
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
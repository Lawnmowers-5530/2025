// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.containers.prod.RobotContainer;


/**
 * The {@link RobotContainer} holds all subsystems, commands, suppliers, etc. in
 * a command-based structure. The only action in {@link Robot} is to schedule
 * the commands defined in {@link RobotContainer}, and to create an instance of
 * the {@link RobotContainer}, which will also bind all buttons to their
 * respective commands.
 */
public class Robot extends TimedRobot {
	//import frc.robot.containers.prod.RobotContainer as Container;
	public static final class Container extends RobotContainer {};

	public static final class RobotGlobalConstants extends frc.robot.constants.RobotGlobal {};

	public Robot() {
		super(RobotGlobalConstants.timePeriodS);

		//TODO: Wpilib shutup for testing, useful to be on during comp
		DriverStation.silenceJoystickConnectionWarning(true);
	}
	private Command autonomousCommand;

	public Container container;

	@Override
	public void robotInit() {
		container = new Container();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		autonomousCommand = container.getAutonomousCommand();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}
}

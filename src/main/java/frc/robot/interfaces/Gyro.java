package frc.robot.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyro<T> {
	public T interfaceGetGyro();

	public Rotation2d interfaceGetRot();

	public double interfaceGetDeg();

	public double interfaceGetRad();

	public void interfaceZeroGyro();

	public double interfaceGetHdgDeg();

	public double interfaceGetHdgRad();
}
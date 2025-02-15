package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    public void setState(SwerveModuleState state);
    public Rotation2d getTurningPosition();
    public double getVelocity();
    public double getOffset();
    public double getDistance();
    public SwerveModulePosition getPosition();
    public SwerveModuleState getState();
}
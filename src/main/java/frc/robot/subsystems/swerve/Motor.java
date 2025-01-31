package frc.robot.subsystems.swerve;

public interface Motor {
    public enum IdleMode {
        Brake,
        Coast
    }
    public void setSpeed(double speed);
    public void setIdleMode(IdleMode mode);
}

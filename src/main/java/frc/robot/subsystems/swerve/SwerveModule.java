package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    Motor drive;
    Motor rotate;
    AbsoluteEncoder encoder;

    PIDController rotatePID;

    public static class Config {
        public Motor drive;
        public Motor rotate;
        public AbsoluteEncoder encoder;
        public double p;
        public double i;
        public double d;
        public double conversionFactor;
        public double angleOffset;
    }

    Config config;

    public SwerveModule(Config config) {
        this.drive = config.drive;
        this.rotate = config.rotate;
        this.encoder = config.encoder;
        this.rotatePID = new PIDController(config.p, config.i, config.d);
    }

    public void setState(SwerveModuleState state) {
        state.optimize(getTurningPosition());
        drive.setSpeed(state.speedMetersPerSecond);
        
        double rotatePIDOutput = rotatePID.calculate(getTurningPosition().getRadians(), state.angle.getRadians());
        rotate.setSpeed(rotatePIDOutput);
        
    }

    public Rotation2d getTurningPosition() {
        return new Rotation2d ( encoder.getAbsoluteAngle() * Math.PI * 2 + this.config.angleOffset );
    }

    public double getVelocity() {
        return encoder.getVelocity() * this.config.conversionFactor / 60;
    }

    public double getAngleOffset() {
        return this.config.angleOffset;
    }

    public double getDistance() {
        return encoder.getPosition() * this.config.conversionFactor;
    }

    public SwerveModulePosition getPos() {
        return new SwerveModulePosition(getDistance(), getTurningPosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getTurningPosition());
    }

    public void setIdleMode(Motor.IdleMode mode) {
        drive.setIdleMode(mode);
        rotate.setIdleMode(mode);
    }
}
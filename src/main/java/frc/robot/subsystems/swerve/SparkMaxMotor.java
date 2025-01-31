package frc.robot.subsystems.swerve;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkMaxMotor implements Motor {
    private SparkMax motor;
    private RelativeEncoder encoder;

    public SparkMaxMotor(int canID) {
        motor = new SparkMax(canID, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        SparkMaxConfig config = new SparkMaxConfig();
        switch (mode) {
            case Brake:
                config.idleMode(SparkBaseConfig.IdleMode.kBrake);
                break;
            case Coast:
                config.idleMode(SparkBaseConfig.IdleMode.kCoast);
                break;
        }
        motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }
}

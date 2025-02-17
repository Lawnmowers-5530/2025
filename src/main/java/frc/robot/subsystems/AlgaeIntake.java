package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class AlgaeIntake extends SubsystemBase {
    static final class AlgaeIntakeConstants extends frc.robot.constants.AlgaeIntake {};

    SparkMax angle;
    SparkMax roller;

    SparkClosedLoopController angleController;


    public AlgaeIntake() {
        angle = new SparkMax(AlgaeIntakeConstants.angleCanId, SparkLowLevel.MotorType.kBrushless);
        roller = new SparkMax(AlgaeIntakeConstants.rollerCanId, SparkLowLevel.MotorType.kBrushless);

        angleController = angle.getClosedLoopController();

        SparkMaxConfig angleConfig = new SparkMaxConfig();
        angleConfig.closedLoop
            .p(AlgaeIntakeConstants.p)
            .i(AlgaeIntakeConstants.i)
            .d(AlgaeIntakeConstants.d);
        angleConfig.closedLoop.maxMotion
            .allowedClosedLoopError(AlgaeIntakeConstants.allowedError)
            .maxAcceleration(AlgaeIntakeConstants.maxAcceleration)
            .maxVelocity(AlgaeIntakeConstants.maxVelocity)
            .positionMode(MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal);

        angle.configure(angleConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public void setSetpoint(double setpoint) {
        angleController.setReference(setpoint, SparkBase.ControlType.kMAXMotionPositionControl);
    }

    public void setRollerPower(double power) {
        roller.set(power);
    }

    public void setManualControl(double power) {
        angle.set(power);
    }

    public void stop() {
        angle.set(0);
        roller.set(0);
    }

    public void intake() {
        roller.set(AlgaeIntakeConstants.rollerPower);
    }

    public void eject() {
        roller.set(-AlgaeIntakeConstants.rollerPower);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled() || DriverStation.isEStopped()) {
            angle.set(0);
            roller.set(0);
        }
    }
}

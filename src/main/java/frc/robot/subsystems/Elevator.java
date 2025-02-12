package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.interfaces.Scoring;

public final class Elevator extends SubsystemBase implements Scoring {
    SparkMax motor1;
    SparkMax motor2;
    SparkMaxConfig motor1Config;
    SparkMaxConfig motor2Config;

    TrapezoidProfile.State goal;

    PIDController elevatorController;

    TrapezoidProfile elevatorProfile;

    double manualSpeed;

    double sp;

    DigitalInput limitSwitch;

    // Inches
    int target;

    public Elevator() {
        target = 0;
        manualSpeed = 0;

        motor1 = new SparkMax(Constants.ElevatorConstants.motor1Id, MotorType.kBrushless);
        motor2 = new SparkMax(Constants.ElevatorConstants.motor2Id, MotorType.kBrushless);

        motor1Config = new SparkMaxConfig();
        motor1Config.inverted(false);
        motor1Config.idleMode(IdleMode.kBrake);
        motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        motor2Config = new SparkMaxConfig();
        motor2Config.inverted(true);
        motor2Config.idleMode(IdleMode.kBrake);
        motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        limitSwitch = new DigitalInput(Constants.ElevatorConstants.limitSwitchChannel);

        elevatorController = new PIDController(Constants.ElevatorConstants.kP1, Constants.ElevatorConstants.kI1, 0);
        elevatorController.setIZone(2);
        elevatorController.setIntegratorRange(-0.2, 0.2);

        elevatorProfile = new TrapezoidProfile(new Constraints(
                Constants.ElevatorConstants.maxVelocity,
                Constants.ElevatorConstants.maxAcceleration));

        sp = getCurrentState().position;
    }

    public void setTarget(double sp) {
        this.sp = sp;
    }

    public TrapezoidProfile.State getCurrentState() {
        double average = (motor1.getEncoder().getPosition() + motor2.getEncoder().getPosition()) / 2.0;
        double averageVel = (motor1.getEncoder().getVelocity() + motor2.getEncoder().getVelocity()) / 2.0;
        return new TrapezoidProfile.State(average, averageVel);

    }

    @Override
    public void periodic() {

        goal.position = target;
        goal.velocity = 0;

        double pud = elevatorController.calculate(getCurrentState().position, sp);
        motor1.set(pud + 0.018);
        motor2.set(pud + 0.018);
    }

    @Deprecated
    public void setDirectSpeed(double speed) {
        motor2.set((speed / 4) + 0.018);
        motor1.set((speed / 4) + 0.018);

    }

    @Override
    public void resetForHang() {
        setTarget(0);
    }

    @Override
    public void goToIntake() {
        setTarget(Constants.ElevatorConstants.inchesForIntake);
    }

    @Override
    public void goToL1() {
        setTarget(Constants.ElevatorConstants.inchesForLevel1);
    }

    @Override
    public void goToL2() {
        setTarget(Constants.ElevatorConstants.inchesForLevel2);
    }

    @Override
    public void goToL3() {
        setTarget(Constants.ElevatorConstants.inchesForLevel3);
    }

    @Override
    public void goToL4() {
        setTarget(Constants.ElevatorConstants.inchesForLevel4);
    }

    public Command goToTarget(int level) {
        return new RunCommand(() -> {
            switch (level) {
                case 0:
                    goToIntake();
                    break;
                case 1:
                    goToL1();
                    break;
                case 2:
                    goToL2();
                    break;
                case 3:
                    goToL3();
                    break;
                case 4:
                    goToL4();
                    break;
                default:
                    break;
            }
        }, this);
    }

    public boolean calibrated = false;

    public Command calibrate() {
        return new RunCommand(() -> {
            if (limitSwitch.get()) {
                motor1.set(Constants.ElevatorConstants.calibrationRaiseSpeed);
                motor2.set(Constants.ElevatorConstants.calibrationLowerSpeed);
            }
        }, this)
                .until(() -> !limitSwitch.get())

                .andThen(() -> {
                    motor1.set(Constants.ElevatorConstants.calibrationLowerSpeed);
                    motor2.set(Constants.ElevatorConstants.calibrationLowerSpeed);
                    if (limitSwitch.get()) {
                        motor1.set(0);
                        motor2.set(0);
                        motor1.getEncoder().setPosition(0);
                        motor2.getEncoder().setPosition(0);

                        SoftLimitConfig limits = new SoftLimitConfig()
                                .forwardSoftLimitEnabled(true)
                                .reverseSoftLimitEnabled(true)
                                .reverseSoftLimit(Constants.ElevatorConstants.calibrationBottomBufferTicks)
                                .forwardSoftLimit(Constants.ElevatorConstants.elevatorRangeTicks
                                        + Constants.ElevatorConstants.calibrationTopBufferTicks);

                        motor1Config.apply(limits);
                        motor2Config.apply(limits);
                        motor1.configure(motor1Config, ResetMode.kNoResetSafeParameters,
                                PersistMode.kPersistParameters);
                        motor2.configure(motor2Config, ResetMode.kNoResetSafeParameters,
                                PersistMode.kPersistParameters);

                        calibrated = true;
                    }
                }, this);
    }
}

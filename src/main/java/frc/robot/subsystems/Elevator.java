package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
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
    private SparkMax motor1;
    private SparkMax motor2;
    private SparkMaxConfig motor1Config;
    private SparkMaxConfig motor2Config;

    private TrapezoidProfile elevatorProfile;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    private ElevatorFeedforward feedforward;

    private double kDt = 0.02;

    private PIDController elevatorController;

    private double sp;

    private DigitalInput limitSwitch;

    public Elevator() {
        feedforward = new ElevatorFeedforward(Constants.ElevatorConstants.kS,
                Constants.ElevatorConstants.kV, Constants.ElevatorConstants.kA);

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

        goal = new TrapezoidProfile.State(0, 0);
        setpoint = new TrapezoidProfile.State(0, 0);
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

        goal.position = sp;
        goal.velocity = 0;

        setpoint = elevatorProfile.calculate(kDt, getCurrentState(), goal);

        double pud = elevatorController.calculate(getCurrentState().position, setpoint.position);
        double ff = feedforward.calculate(setpoint.velocity);
        motor1.set(pud + ff);
        motor2.set(pud + ff);
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
        setTarget(Constants.ElevatorConstants.intake);
    }

    @Override
    public void goToL1() {
        setTarget(Constants.ElevatorConstants.level1);
    }

    @Override
    public void goToL2() {
        setTarget(Constants.ElevatorConstants.level2);
    }

    @Override
    public void goToL3() {
        setTarget(Constants.ElevatorConstants.level3);
    }

    @Override
    public void goToL4() {
        setTarget(Constants.ElevatorConstants.level4);
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
                motor1.set(Constants.ElevatorConstants.calibrationSpeed);
                motor2.set(Constants.ElevatorConstants.calibrationSpeed);
            }
        }, this)
                .until(() -> !limitSwitch.get())

                .andThen(() -> {
                    motor1.set(Constants.ElevatorConstants.calibrationSpeed);
                    motor2.set(Constants.ElevatorConstants.calibrationSpeed);
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

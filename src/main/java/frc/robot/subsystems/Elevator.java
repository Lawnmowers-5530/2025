package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;

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
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public final class Elevator extends SubsystemBase {
    //import frc.robot.constants.Elevator as ElevatorConstants
    static final class ElevatorConstants extends frc.robot.constants.Elevator {};


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

    private SysIdRoutine routine;
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    public Elevator() {
       // routine = new SysIdRoutine(
       //     new SysIdRoutine.Config(Velocity.ofBaseUnits(3, VelocityUnit.combine(Volts, Seconds)), Voltage.ofBaseUnits(3, Volts), Time.ofBaseUnits(5, Second)),
       //         new SysIdRoutine.Mechanism(this::voltageDrive,
       //                 log -> {
       //                     // Record a frame for the left motors. Since these share an encoder, we consider
       //                     // the entire group to be one motor.
       //                     log.motor("motor1")
       //                             .voltage(
       //                                     m_appliedVoltage.mut_replace(
       //                                             motor1.get() * RobotController.getBatteryVoltage(), Volts))
       //                             .linearPosition(m_distance.mut_replace(motor1.getEncoder().getPosition(), Meters))
       //                             .linearVelocity(
       //                                     m_velocity.mut_replace(motor1.getEncoder().getVelocity(), MetersPerSecond));
       //                     // Record a frame for the right motors. Since these share an encoder, we
       //                     // consider
       //                     // the entire group to be one motor.
       //                     log.motor("motor2")
       //                             .voltage(
       //                                     m_appliedVoltage.mut_replace(
       //                                             motor2.get() * RobotController.getBatteryVoltage(), Volts))
       //                             .linearPosition(m_distance.mut_replace(motor2.getEncoder().getPosition(), Meters))
       //                             .linearVelocity(
       //                                     m_velocity.mut_replace(motor2.getEncoder().getVelocity(), MetersPerSecond));
       //                 },
       //                 // Tell SysId to make generated commands require this subsystem, suffix test
       //                 // state in
       //                 // WPILog with this subsystem's name ("drive")
       //                 this));

        feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kV, ElevatorConstants.kA);

        motor1 = new SparkMax(ElevatorConstants.motor1Id, MotorType.kBrushless);
        motor2 = new SparkMax(ElevatorConstants.motor2Id, MotorType.kBrushless);

        motor1Config = new SparkMaxConfig();
        motor1Config.inverted(false);
        motor1Config.idleMode(IdleMode.kBrake);
        motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        motor2Config = new SparkMaxConfig();
        motor2Config.inverted(true);
        motor2Config.idleMode(IdleMode.kBrake);
        motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        limitSwitch = new DigitalInput(ElevatorConstants.limitSwitchChannel);

        elevatorController = new PIDController(ElevatorConstants.kP1, ElevatorConstants.kI1, 0);
        elevatorController.setIZone(2);
        elevatorController.setIntegratorRange(-0.2, 0.2);

        elevatorProfile = new TrapezoidProfile(new Constraints(
                ElevatorConstants.maxVelocity,
                ElevatorConstants.maxAcceleration));

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

    public void voltageDrive(Voltage voltage) {
        motor1.setVoltage(voltage);
        motor2.setVoltage(voltage);
    }

    public void resetForHang() {
        setTarget(0);
    }

    public void goToIntake() {
        setTarget(ElevatorConstants.intake);
    }

    public void goToL1() {
        setTarget(ElevatorConstants.level1);
    }

    public void goToL2() {
        setTarget(ElevatorConstants.level2);
    }

    public void goToL3() {
        setTarget(ElevatorConstants.level3);
    }

    public void goToL4() {
        setTarget(ElevatorConstants.level4);
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
                motor1.set(ElevatorConstants.calibrationSpeed);
                motor2.set(ElevatorConstants.calibrationSpeed);
            }
        }, this)
                .until(() -> !limitSwitch.get())

                .andThen(() -> {
                    motor1.set(ElevatorConstants.calibrationSpeed);
                    motor2.set(ElevatorConstants.calibrationSpeed);
                    if (limitSwitch.get()) {
                        motor1.set(0);
                        motor2.set(0);
                        motor1.getEncoder().setPosition(0);
                        motor2.getEncoder().setPosition(0);

                        SoftLimitConfig limits = new SoftLimitConfig()
                                .forwardSoftLimitEnabled(true)
                                .reverseSoftLimitEnabled(true)
                                .reverseSoftLimit(ElevatorConstants.calibrationBottomBufferTicks)
                                .forwardSoftLimit(ElevatorConstants.elevatorRangeTicks
                                        + ElevatorConstants.calibrationTopBufferTicks);

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

    ///**
    // * Returns a command that will execute a quasistatic test in the given
    // * direction.
    // *
    // * @param direction The direction (forward or reverse) to run the test in
    // */
    //public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //    return routine.quasistatic(direction);
    //}
//
    ///**
    // * Returns a command that will execute a dynamic test in the given direction.
    // *
    // * @param direction The direction (forward or reverse) to run the test in
    // */
    //public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //    return routine.dynamic(direction);
    //}
}

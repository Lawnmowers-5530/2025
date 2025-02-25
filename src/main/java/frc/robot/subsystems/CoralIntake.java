package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralIntake.States;

public class CoralIntake extends SubsystemBase {
    //import frc.robot.constants.CoralIntake.Pivot as PivotConstants;
    static final class PivotConstants extends frc.robot.constants.CoralIntake.Pivot {};

    private SparkMax intake;
    private SparkMax pivot;
    private PIDController pivotController;
    private SimpleMotorFeedforward pivotFeedforward;
    private SparkMaxConfig intakeConfig;
    private SparkMaxConfig pivotConfig;
    private LaserCan fakeBeamBreak;
    private LaserCan fakeBeamBreak2;
    private AbsoluteEncoder pivotEncoder;
    public States state = States.HAS_CORAL;


    public double target = PivotConstants.bottomPos;

    boolean laserCanSwitch = false;

    public CoralIntake() {
        intakeConfig = new SparkMaxConfig();
        intakeConfig.smartCurrentLimit(20,20);


        pivotConfig = new SparkMaxConfig();
        pivotConfig.inverted(false);
        //intakeConfig.softLimit.
        fakeBeamBreak = new LaserCan(PivotConstants.laserCan1Id);
        fakeBeamBreak2 = new LaserCan(PivotConstants.laserCan2Id);
        
        intake = new SparkMax(PivotConstants.intakeId, MotorType.kBrushless);
        intake.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        pivot = new SparkMax(PivotConstants.pivotId, MotorType.kBrushless);
        pivot.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        pivotEncoder = pivot.getAbsoluteEncoder();
        pivotController = new PIDController(PivotConstants.Kp, 0, 0);
        pivotController.setTolerance(PivotConstants.tolerance);
        

    }
    @Deprecated
    public double uff(double theta) {
        double cosTheta = Math.cos(theta);
        double numer = (PivotConstants.length * PivotConstants.restistance * PivotConstants.mass * PivotConstants.gravity);
        double denom = (PivotConstants.center * PivotConstants.gearRatio * PivotConstants.kt);
        return numer * cosTheta  * denom;
    }
    @Override
    public void periodic() {
        double out = pivotController.calculate(pivotEncoder.getPosition(), Math.min(PivotConstants.bottomPos, Math.max(PivotConstants.topPos, target)));
        pivot.set(out);
        SmartDashboard.putNumber("Pivot out", out);
        SmartDashboard.putNumber("pivot position", pivot.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("pivot target", this.target);
        SmartDashboard.putNumber("lasercan", fakeBeamBreak.getMeasurement().distance_mm);
        SmartDashboard.putNumber("intake speed", intake.get());
    }

    public BooleanSupplier atTarget = () -> {
        return Math.abs(this.target-this.pivot.getAbsoluteEncoder().getPosition())<PivotConstants.tolerance;

    };

    public void manualPivot(double speed) {
        this.target += speed/100;
        this.target = Math.max(this.target, 0.67);
        this.target = Math.min(this.target, 0.9);
    }

    public void setTarget(Targets target) {
        switch (target) {
            case INTAKE:
            this.target = PivotConstants.intakePos;
            break;
            case BOTTOM:
            this.target = PivotConstants.bottomPos;
            break;
            case MIDDLE:
            this.target = PivotConstants.middlePos;
            break;
            case TOP:
            this.target = PivotConstants.topPos;
            break;
            case L4:
            this.target = PivotConstants.L4;

        }
    }

    public Command anglePivot(Targets target) {
        return new InstantCommand(() -> {
            state = States.NOT_HAS_AND_NOT_WANTS;
            setTarget(target);
        }, this);
    }

    public enum Targets {
        INTAKE, BOTTOM, MIDDLE, TOP, L4
    }
    public void intake() {
        System.out.println("intaking");
        intake.set(PivotConstants.intakePower);
        state = States.WANTS_CORAL;
    }
    public void outtake() {
        intake.set(PivotConstants.intakePower);
    }

    public void stopIntake() {
        intake.set(0);
    }

    public boolean coralDetected1() {
        state = States.HAS_CORAL;
        double measurement = fakeBeamBreak.getMeasurement().distance_mm;
        SmartDashboard.putNumber("measurement", measurement);
        return measurement < 35;
    }
    public boolean notCoralDetected1() {
        return !coralDetected1();
    }
    public boolean notCoralDetected() {
        return laserCanSwitch? !coralDetected1() : !coralDetected2();
    }
    private boolean coralDetected2() {
        var measurement = fakeBeamBreak2.getMeasurement();
        return measurement.distance_mm < 35;
    }
    public boolean coralDetected() {
        return laserCanSwitch ? coralDetected1() : coralDetected2();
    }
    public void setLaserCanSwitch(boolean value) {
        laserCanSwitch = value;
    }
    public static class Helper {
        public static  double ticksToDegree(double ticks) {
            return ticks * 360;//domath
        }
        
    }

    public Command intakeCommand() {
        return new RunCommand(() -> {
            intake();
        }, this);
    }

    public Command stopIntakeCommand() {
        return new InstantCommand(() -> {
            stopIntake();
        }, this);
    }
    

    public enum States {
        HAS_CORAL, WANTS_CORAL, NOT_HAS_AND_NOT_WANTS
    }

}

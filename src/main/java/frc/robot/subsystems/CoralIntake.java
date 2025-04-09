package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class CoralIntake extends SubsystemBase {
    // import frc.robot.constants.CoralIntake.Pivot as PivotConstants;
    static final class PivotConstants extends frc.robot.constants.CoralIntake.Pivot {
    };

    private SparkMax intake;
    private SparkMax pivot;
    private PIDController pivotController;
    private SparkMaxConfig intakeConfig;
    private SparkMaxConfig pivotConfig;
    private LaserCan fakeBeamBreak;
    private LaserCan fakeBeamBreak2;
    LaserCan inFunnel;
    private AbsoluteEncoder pivotEncoder;
    public States state = States.HAS_CORAL;
    

    public double target = PivotConstants.bottomPos;

    boolean laserCanSwitch = false;

    int primaryLaserCANFailCount = 0;
    int secondaryLaserCANFailCount = 0;

    public CoralIntake() {
        intakeConfig = new SparkMaxConfig();
        intakeConfig.smartCurrentLimit(20, 20);

        pivotConfig = new SparkMaxConfig();

        pivotConfig.closedLoop.pidf(PivotConstants.Kp, 0, 0, PivotConstants.ff);
        pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        pivotConfig.closedLoop.positionWrappingEnabled(true).positionWrappingInputRange(0, 1);
        pivotConfig.inverted(true);
        pivotConfig.absoluteEncoder.inverted(true);
        // intakeConfig.softLimit.
        fakeBeamBreak = new LaserCan(PivotConstants.laserCan1Id);
        fakeBeamBreak2 = new LaserCan(PivotConstants.laserCan2Id);
        inFunnel = new LaserCan(28);
        intake = new SparkMax(PivotConstants.intakeId, MotorType.kBrushless);
        intake.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        pivot = new SparkMax(PivotConstants.pivotId, MotorType.kBrushless);
        pivot.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        pivotEncoder = pivot.getAbsoluteEncoder();
        pivotController = new PIDController(PivotConstants.Kp, 0, 0);
        pivotController.setTolerance(PivotConstants.tolerance);

    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake power", intake.get());
        pivot.getClosedLoopController().setReference(target, ControlType.kPosition);
    
        if (intake.get() != 0) {
            state = States.WANTS_CORAL;
        } else if (legacyCoralDetected()) {
            state = States.HAS_CORAL;
        } else {
            state = States.IDLE;
        }
        //SmartDashboard.putBoolean("is coral in effector", notCoralDetected());

        SmartDashboard.putNumber("pivot position", pivot.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("pivot target", this.target);
        //SmartDashboard.putNumber("lasercan", fakeBeamBreak.getMeasurement().distance_mm);
        SmartDashboard.putNumber("intake speed", intake.get());
    }

    public BooleanSupplier atTarget = () -> {
       
        return Math.abs(this.target - this.pivot.getAbsoluteEncoder().getPosition()) < PivotConstants.tolerance;

    };

    public void manualPivot(double speed) {
        //this.target += speed / 100;
        //this.target = Math.max(this.target, 0.67);
        //if (this.target > 0.9) {
          //  this.target = Math.max(this.target, 0.9);
        //}else {
          //  this.target = Math.min(this.target, 0.3);


        //}
        
    }
    public boolean coralInFunnel() {
        if (inFunnel.getMeasurement() == null) {
            System.err.println("Funnel sensor failure");
            return false;
        }
        return inFunnel.getMeasurement().distance_mm < 300;
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
            setTarget(target);
        }, this);
    }

    public enum Targets {
        INTAKE, BOTTOM, MIDDLE, TOP, L4
    }

    public void intake() {
       
        intake.set(PivotConstants.intakePower);
    }

    public void outtake() {
        intake.set(PivotConstants.outtakePower);
    }

    public void outtakeL1() {
        intake.set(PivotConstants.outtakeL1);
    }

    public void stopIntake() {
        intake.set(0);
    }

    @Deprecated
    public boolean coralDetected1() {
        var measurement = fakeBeamBreak.getMeasurement();
        if (measurement == null) {
            System.out.println("Sensor 1 failure");
            return false;
        }
        return measurement.distance_mm < 35;
    
    }

    public boolean coralDetected() {
        boolean primaryFailed = false;
        boolean secondaryFailed = false;

        var primaryMeasurement = laserCanSwitch ? fakeBeamBreak2.getMeasurement(): fakeBeamBreak.getMeasurement();
        if (primaryMeasurement == null) {
            primaryLaserCANFailCount++;
            primaryFailed = true;
            SmartDashboard.putNumber("Primary Lazercan fail count", primaryLaserCANFailCount);
        } else {
            return primaryMeasurement.distance_mm < 35;
        }

        var secondaryMeasurement = laserCanSwitch? fakeBeamBreak.getMeasurement(): fakeBeamBreak2.getMeasurement();
        if (secondaryMeasurement == null) {
            secondaryLaserCANFailCount++;
            secondaryFailed = true;
            SmartDashboard.putNumber("Secondary Lazercan fail count:", secondaryLaserCANFailCount);
        } else {
            return secondaryMeasurement.distance_mm < 35;
        }

        System.err.println("NO VAILD LASERCAN OUTPUT, INTAKE WILL NOT WORK, fail count 1: " + primaryLaserCANFailCount + ", fail count 2: " + secondaryLaserCANFailCount);
        return true;
    }

    public boolean notCoralDetected() {
        return !coralDetected();
    }

    @Deprecated
    public boolean notCoralDetected1() {
        return !coralDetected1();
    }

    @Deprecated
    public boolean legacyNotCoralDetected() {
        return laserCanSwitch ? !coralDetected2() : !coralDetected1();
    }

    @Deprecated
    private boolean coralDetected2() {

        var measurement = fakeBeamBreak2.getMeasurement();
        if (measurement == null) {
            System.out.println("Sensor 2 failure");
            return false;
        }
        return measurement.distance_mm < 35;
    }

    @Deprecated
    public boolean legacyCoralDetected() {
        return laserCanSwitch ? coralDetected2() : coralDetected1();
    }

    public void setLaserCanSwitch(boolean value) {
        laserCanSwitch = value;
    }

    public static class Helper {
        public static double ticksToDegree(double ticks) {
            return ticks * 360;// domath
        } // TODO: can remove?

    }

    public Command intakeCommand() {
        return new RunCommand(() -> {
            intake();
        }, this);
    }

    public Command outtakeL1Command() {
        return new RunCommand(
                () -> {
                    outtakeL1();
                },
                this);
    }

    public Command stopIntakeCommand() {
        return new InstantCommand(() -> {
            stopIntake();
        }, this);
    }
    public Command waitUntilCoralInFunnel() {
        return new WaitUntilCommand(this::coralInFunnel);
    }

    public enum States {
        HAS_CORAL, WANTS_CORAL, IDLE
    }

}

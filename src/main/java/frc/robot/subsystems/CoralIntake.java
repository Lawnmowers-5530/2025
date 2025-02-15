package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        double out = pivotController.calculate(pivotEncoder.getPosition(), target);
        pivot.set(out);
        SmartDashboard.putNumber("Pivot out", out);
        SmartDashboard.putNumber("position", pivot.getAbsoluteEncoder().getPosition());

        


    }

    public void manualPivot(double speed) {
        pivot.set(speed/5);
        SmartDashboard.putNumber("joy speed", speed/5);
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

        }
    }
    public enum Targets {
        INTAKE, BOTTOM, MIDDLE, TOP
    }
    public void setPivotSpeed(double speed) {
        pivot.set(speed);
    }
    public void intake() {
        intake.set(PivotConstants.intakePower);
    }
    public void outtake() {
        intake.set(-PivotConstants.intakePower);
    }

    public void stopIntake() {
        intake.set(0);
    }

    public boolean coralDetected1() {
        double measurement = fakeBeamBreak.getMeasurement().distance_mm;
        SmartDashboard.putNumber("measurement", measurement);
        System.out.println("looping?");
        return measurement < 20;
    }
    private boolean coralDetected2() {
        var measurement = fakeBeamBreak2.getMeasurement();
        return measurement.distance_mm < 20;
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

}

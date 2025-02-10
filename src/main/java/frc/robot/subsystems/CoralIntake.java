package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.apriltag.jni.AprilTagJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
    SparkMax intake;
    SparkMax pivot;
    PIDController pivotController;
    SimpleMotorFeedforward pivotFeedforward; 
    SparkMaxConfig intakeConfig;
    LaserCan fakeBeamBreak;
    LaserCan fakeBeamBreak2;
    public double target = 0;

    boolean laserCanSwitch = false;

    public CoralIntake() {
        intakeConfig = new SparkMaxConfig();
        intakeConfig.smartCurrentLimit(20,20);
        //intakeConfig.softLimit.
        fakeBeamBreak = new LaserCan(Constants.PivotConstants.laserCan1Id);
        fakeBeamBreak2 = new LaserCan(Constants.PivotConstants.laserCan2Id);
        
        intake = new SparkMax(Constants.PivotConstants.intakeId, MotorType.kBrushless);
        intake.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        pivot = new SparkMax(Constants.PivotConstants.pivotId, MotorType.kBrushless);
        pivotController = new PIDController(Constants.PivotConstants.Kp, 0, 0);
     
    }
    public double uff(double theta) {
        double cosTheta = Math.cos(theta);
        double numer = (Constants.PivotConstants.length * Constants.PivotConstants.restistance * Constants.PivotConstants.mass * Constants.PivotConstants.gravity);
        double denom = (Constants.PivotConstants.center * Constants.PivotConstants.gearRatio * Constants.PivotConstants.kt);
        return numer * cosTheta  * denom;
    }
    @Override
    public void periodic() {
        double out = pivotController.calculate(target,Helper.ticksToDegree(pivot.getAbsoluteEncoder().getPosition())) + uff(Helper.ticksToDegree(pivot.getAbsoluteEncoder().getPosition()));
        SmartDashboard.putNumber("Pivot out", out);
        SmartDashboard.putNumber("position", pivot.getAbsoluteEncoder().getPosition());

        


    }

    public void setTarget(Targets target) {
        switch (target) {
            case INTAKE:
            this.target = Constants.PivotConstants.intakePos;
            break;
            case BOTTOM:
            this.target = Constants.PivotConstants.bottomPos;
            break;
            case MIDDLE:
            this.target = Constants.PivotConstants.middlePos;
            break;
            case TOP:
            this.target = Constants.PivotConstants.topPos;
            break;

        }
    }
    public enum Targets {
        INTAKE, BOTTOM, MIDDLE, TOP
    }
    public void setPivotSpeed(double speed) {
        pivot.set(speed / 3);
    }
    public void intake() {
        intake.set(Constants.PivotConstants.intakePower);
    }
    public void outtake() {
        intake.set(-Constants.PivotConstants.intakePower);
    }

    public void stopIntake() {
        intake.set(0);
    }
    public boolean coralDetected1() {
        double measurement = fakeBeamBreak.getMeasurement().distance_mm;
        SmartDashboard.putNumber("measurement", measurement);
        System.out.println("looping?");
        if (measurement < 20) {
            return true;
        } else {
            return false;
        }
    }
    private boolean coralDetected2() {
        var measurement = fakeBeamBreak2.getMeasurement();
        if (measurement.distance_mm < 20) {
            return true;
        } else {
            return false;
        }
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

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.interfaces.Scoring;


public final class Elevator extends SubsystemBase implements Scoring{
    SparkMax motor1;
    SparkMax motor2;
    SparkMaxConfig motor1Config;
    SparkMaxConfig motor2Config;
    PIDController elevatorController;

    SimpleMotorFeedforward feedforward;
    
    TrapezoidProfile elevatorProfile;

    double manualSpeed;


    //inches
    TrapezoidProfile.State goal;

    //Inches
    double sp;

    //DigitalInput limitSwitch;

    //KalmanFilter<N1, N2, N1> filter; Too lazy to get
    
    //Inches
    int target = 0;



    
    
    
    public Elevator() {
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


        

        //limitSwitch = new DigitalInput(Constants.ElevatorConstants.limitSwitchChannel);

        elevatorController = new PIDController(Constants.ElevatorConstants.kP1, Constants.ElevatorConstants.kI1, 0);
        elevatorController.setIZone(2);
        elevatorController.setIntegratorRange(-0.2, 0.2);
        elevatorProfile = new TrapezoidProfile(new Constraints(
            //Inches
            Constants.ElevatorConstants.maxVelocity,
            //Inches
            Constants.ElevatorConstants.maxAcceleration
        ));
        sp = getCurrentState().position;
        goal = new TrapezoidProfile.State(target, 0);
        //feedforward = new SimpleMotorFeedforward(0, 0);
        
        //filter = new KalmanFilter<>(null, null, null, null, null, target);


    }
    public void setTarget(double sp) {
        this.sp = sp;
    }
   
    public TrapezoidProfile.State getCurrentState() {
        double m1ticks = motor1.getEncoder().getPosition();
        double m2ticks = motor2.getEncoder().getPosition();
        double average = Helpers.ticks2inches((m1ticks + m2ticks)/2.0);
        double averageVel = Helpers.ticks2inches((motor1.getEncoder().getVelocity() + motor2.getEncoder().getVelocity())/2.0);
        return new TrapezoidProfile.State(average, averageVel);

    }
    @Override
    public void periodic() {

            goal.position = target;
            goal.velocity = 0;
            //setpoint = getCurrentState();
            
            //setpoint = elevatorProfile.calculate(0.02, setpoint, goal);

            double pud = elevatorController.calculate(getCurrentState().position,sp);
            //System.out.println("pud: " + pud);
            System.out.println(this.getCurrentState().position);
            motor1.set(pud);
            motor2.set(pud);

            //double feed = feedforward.calculate(setpoint.velocity);
            //double[] states = {pud + feed, goal.position, getCurrentState().position};
            //SmartDashboard.putNumberArray("OUT, GOAL, CURRENT", states);
    }
    double speed;

    @Deprecated
    public void setDirectSpeed(double speed) {
        motor2.set((speed/4) +0.018);
        motor1.set((speed/4) +0.018);
       // System.out.println(getCurrentState().position);

    }
   
  
    




    private static class Helpers {
        public static double inches2Ticks(double inches) {
            return Constants.ElevatorConstants.ticksPerInch * inches;
        }
        public static double ticks2inches(double ticks) {
            return ticks/Constants.ElevatorConstants.ticksPerInch;
        }
    }




    @Override
    public void goToL1() {
        setTarget(Constants.ElevatorConstants.inchesForLevel1);
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
    public void goToL4() {
        setTarget(Constants.ElevatorConstants.inchesForLevel4);
    }
    @Override
    public void goToL3() {
        setTarget(Constants.ElevatorConstants.inchesForLevel3);
    }
    @Override
    public void goToL2() {
        setTarget(Constants.ElevatorConstants.inchesForLevel2);
    }
}

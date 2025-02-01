package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.interfaces.Scoring;


public final class Elevator extends SubsystemBase implements Scoring{
    SparkMax motor1;
    SparkMax motor2;
    SparkMaxConfig motor1Config;
    PIDController m1Controller;
    PIDController m2Controller;
    SimpleMotorFeedforward m1Feedforward;
    SimpleMotorFeedforward m2Feedforward;
    TrapezoidProfile elevatorProfile;
    TrapezoidProfile.State goal;
    TrapezoidProfile.State setpoint;
    DigitalInput limitSwitch;
    //KalmanFilter<N1, N2, N1> filter; Too lazy to get
    
    int target = 0;
    boolean resetting = false;
    
    
    public Elevator() {
        motor1 = new SparkMax(Constants.ElevatorConstants.motor1Id, MotorType.kBrushless);
        motor2 = new SparkMax(Constants.ElevatorConstants.motor2Id, MotorType.kBrushless);
        motor1Config = new SparkMaxConfig();
        motor1Config.inverted(true);
        motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        

        limitSwitch = new DigitalInput(Constants.ElevatorConstants.limitSwitchChannel);

        m1Controller = new PIDController(Constants.ElevatorConstants.kP1, Constants.ElevatorConstants.kI1, 0);
        m2Controller = new PIDController(Constants.ElevatorConstants.kP2, 0, Constants.ElevatorConstants.kI2);
        elevatorProfile = new TrapezoidProfile(new Constraints(
            Constants.ElevatorConstants.maxVelocity,
            Constants.ElevatorConstants.maxAcceleration
        ));
        setpoint = getCurrentState();
        goal = new TrapezoidProfile.State(target, 0);
        m1Feedforward = new SimpleMotorFeedforward(0, 0);
        m2Feedforward = new SimpleMotorFeedforward(0, 0);
        //filter = new KalmanFilter<>(null, null, null, null, null, target);


    }
    public void setTarget(int target) {
        this.target = target;
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
        if (!resetting) {
            goal.position = target;
            setpoint = getCurrentState();
            
            setpoint = elevatorProfile.calculate(0.2, setpoint, goal);

            double pos1 = m1Controller.calculate(motor1.getEncoder().getPosition(), setpoint.position);
            double vel1 = m1Feedforward.calculate(setpoint.velocity);
            motor1.setVoltage(pos1 + vel1);

            double pos2 = m2Controller.calculate(motor2.getEncoder().getPosition(), setpoint.position);
            double vel2 = m2Feedforward.calculate(setpoint.velocity);
            motor1.setVoltage(pos2 + vel2);
        }else {
            if (limitSwitch.get()) {
                motor1.setVoltage(-1/12.0);
                motor2.setVoltage(-1/12.0);
            }else {
                motor1.getEncoder().setPosition(0);
                motor2.getEncoder().setPosition(0);
                motor1.setVoltage(0);
                motor2.setVoltage(0);
                resetting= false;
            }
           
        }

        

        

    }
    public void reset() {
        resetting = true;
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

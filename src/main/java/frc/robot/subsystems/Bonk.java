package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Carbon fiber algae knocker subsystem
 */
public class Bonk extends SubsystemBase{
    SparkMax bonker;


    private static Bonk instance;
    public static Bonk getInstance() {
        return instance;
    }

    //Constants
    private final int id = 25;
    private static final double upPos = 38.3;
    private static final double downPos = 21.7;
    private static final double middlePos = 34.2;
    private static final double  resetPos = 0;
    private static final double Kp=0.42; 
    private static final double Ki=0; 
    private static final double Kd = 0;
    private static final double ff = 0.00;



   
    private double setpoint;


    public enum Targets {
       
        UP(upPos),
        DOWN(downPos),
        MIDDLE(middlePos),
        RESET(resetPos);



        private double val;
        Targets(double pos) {
            this.val = pos;
        }
        public double getPos() {
            return val;
        }

    }

    public void setTarget(Targets target) {
        setpoint = target.getPos();
    }

    public Bonk(){
       
        if (instance == null) {
            setpoint = resetPos;
            bonker = new SparkMax(id, MotorType.kBrushless);
            bonker.getEncoder().setPosition(0);
            ClosedLoopConfig config = new ClosedLoopConfig().p(Kp).i(Ki).d(Kd).velocityFF(ff);
            //config.maxOutput(0.75).minOutput(-0.75);
            SparkMaxConfig confi = new SparkMaxConfig();
            confi.apply(config);
            bonker.configure(confi,ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            instance = this;
        }
       
    
    }
 
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pos", bonker.getEncoder().getPosition());
        bonker.getClosedLoopController().setReference(setpoint, ControlType.kPosition);
    }
    
}

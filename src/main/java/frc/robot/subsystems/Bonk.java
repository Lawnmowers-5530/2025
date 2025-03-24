package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    private final int id = 50;
    private static final double endPos = -4.14;
    private static final double upPos = 2;
    private static final double downPos = 4;
    private static final double middlePos = 3;
    private static final double  resetPos = 0;
    private static final double Kp=0.1; 
    private static final double Ki=0; 
    private static final double Kd = 0;



    PIDController bonkController;
    private double setpoint;


    public enum Targets {
        END(endPos),
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
            bonkController = new PIDController(Kp, Ki, Kd);
            instance = this;
        }
       
    
    }
 
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pos", bonker.getEncoder().getPosition());
        //bonker.set(bonkController.calculate(bonker.getEncoder().getPosition(), setpoint));
    }
    
}

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends  SubsystemBase {
    public static final class HangConstants  {
        public static final int leftMotorChannel = 17;
        
        public static final int servoLeftPWMId = 0;
        
        public static  final float servoReleaseLeft = 0;
        public static final float servoHoldLeft = 0;
    
        public static final double hangPower = 0.15;
        public static final double toHangPos = -0.2;
        public static final double doneHangPos = 0;
        public static final int funnelRelease = 1;
    }

    //#region
    SparkMax leftHang;
    Servo funnelRelease;
    
    Servo releaseLeft;
    
    private boolean release = false;
    private boolean releaseFunnel = false;
  
    //#endregion
    public Hang() {

        leftHang = new SparkMax(HangConstants.leftMotorChannel, MotorType.kBrushless);
        funnelRelease = new Servo(HangConstants.funnelRelease);
        releaseLeft = new Servo(HangConstants.servoLeftPWMId);
    }
    public Command autoHang() {
        return new RunCommand(()-> {
            leftHang.set(HangConstants.hangPower);
        }, this).until(this::isHanged).andThen(new RunCommand(()->stop(), this));
    }
    public Command autoOut() {
        return new RunCommand(()-> {
            leftHang.set(-HangConstants.hangPower);
        }, this).until(this::isUnhinged).andThen(new RunCommand(()->stop(), this));
    }
    public void stop() {
        leftHang.set(0);
    }
    public boolean isHanged() {
        return (leftHang.getEncoder().getPosition() > HangConstants.doneHangPos);

    }
    public boolean isUnhinged() {
        return (leftHang.getEncoder().getPosition() < HangConstants.toHangPos);
    }
    public void manualInput(double input, double input2) {
        if (input < -0.01){
            
            leftHang.set(input+0.01);
            release();
        }else {
            hold();
            leftHang.set(input);
           
        }

    }
    
    public Command toggleRelease() {
        return new RunCommand(()-> {
            toggleServos();
        }, this);

    }
    public Command toggleFunnel() {
        return new RunCommand(()-> {
            releaseFunnel = !releaseFunnel;
        }, this);
    }
    public void toggleServos() {
        if (release) {
            releaseLeft.set(0.017);
            release = false;
            
        }else{
            releaseLeft.set(0.15);
            release = true;
        }
    }
    public void release() {
        release = true;
        releaseLeft.set(0.017);
    }
    public void hold() {
        releaseLeft.set(0.15);
        release = false;
    }
    @Override 
    public void periodic() {
        SmartDashboard.putNumber("Hang Pos", leftHang.getEncoder().getPosition());
        funnelRelease.set(releaseFunnel ? 1: 0);
     
        
    }
   

  
}


    
  


package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hang extends  SubsystemBase {
    //#region
    SparkMax leftHang;
    
    Servo releaseLeft;
    
    private boolean release = false;
  
    //#endregion
    public Hang() {
        leftHang = new SparkMax(Constants.HangConstants.leftMotorChannel, MotorType.kBrushless);
       
        releaseLeft = new Servo(Constants.HangConstants.servoLeftPWMId);
    }
    public Command autoHang() {
        return new RunCommand(()-> {
            leftHang.set(Constants.HangConstants.hangPower);
        }, this).until(this::isHanged).andThen(new RunCommand(()->stop(), this));
    }
    public Command autoOut() {
        return new RunCommand(()-> {
            leftHang.set(-Constants.HangConstants.hangPower);
        }, this).until(this::isUnhinged).andThen(new RunCommand(()->stop(), this));
    }
    public void stop() {
        leftHang.set(0);
    }
    public boolean isHanged() {
        return (leftHang.getEncoder().getPosition() > Constants.HangConstants.doneHangPos);

    }
    public boolean isUnhinged() {
        return (leftHang.getEncoder().getPosition() < Constants.HangConstants.toHangPos);
    }
    public void manualInput(double input) {
        if (input < 0){
            releaseLeft.set(input);
            release();
        }else {
            hold();
            releaseLeft.set(input);
        }
    }
    public Command toggleRelease() {
        return new RunCommand(()-> {
            toggleServos();
        }, this);

    }
    public void toggleServos() {
        if (release) {
            releaseLeft.set(0.15);
            release = false;
            
        }else{
            releaseLeft.set(0.017);
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
     
        
    }
   

  
}


    
  


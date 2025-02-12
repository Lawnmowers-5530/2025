package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Hang extends  SubsystemBase {
    //#region
    SparkMax leftHang;
    SparkMax rightHang;
    Servo releaseLeft;
    Servo releaseRight;
    private boolean release = false;
    private boolean manual = false;
    public boolean hanged = false;
    //#endregion
    public Hang() {
        leftHang = new SparkMax(Constants.HangConstants.leftMotorChannel, MotorType.kBrushless);
        rightHang = new SparkMax(Constants.HangConstants.rightMotorChannel, MotorType.kBrushless);
        releaseLeft = new Servo(Constants.HangConstants.servoLeftPWMId);
        releaseRight = new Servo(Constants.HangConstants.servoRightPWMId);
    }
    public void hangDown() {
        leftHang.set(-Constants.HangConstants.hangPower);
        rightHang.set(Constants.HangConstants.hangPower);

    }
    public void stop() {
        leftHang.set(0);
        rightHang.set(0);
    }
    public void hangUp() {
        leftHang.set(Constants.HangConstants.hangPower);
        rightHang.set(-Constants.HangConstants.hangPower);
    }

    public boolean hangAtZero() {
        return leftHang.getEncoder().getPosition() < 5.0 || rightHang.getEncoder().getPosition() <= 5.0;
    }

    public void toggleRelease() {
        release = !release;
    }
    public void releaseRatchet() {
        release = false;
    }
    public void stopRatchet() {
        release = true;
    }
    public void toggleManual() {
        manual = !manual;
    }
    public void setPowerUsingManual(float manualInput) {
        if (manual) {
            leftHang.set(release ? -manualInput * Constants.HangConstants.hangPower : Math.min(0, -manualInput * Constants.HangConstants.hangPower));
            rightHang.set(release ? manualInput * Constants.HangConstants.hangPower : Math.max(0, manualInput * Constants.HangConstants.hangPower));
        }
    }


    @Override 
    public void periodic() {
        releaseLeft.set(release ? 0.017 : 0.15);
        releaseRight.set(release ? 0.16 : 0);
     
        
    }
    //#region Commands
    public Command autoHang() {
        return new RunCommand(()->{release=false;hangDown();}, this).andThen(new WaitCommand(Constants.HangConstants.hangTime)).andThen(()->stop());
    }
    public Command releaseToZero() {
        return new RunCommand(()-> {release=true;hangUp();}, this).until(this::hangAtZero).andThen(()->stop());
    }
    public Command hardStop() {
        return new RunCommand(()->stop(), this);
    }


    //#endregion
}

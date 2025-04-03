package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase {
    public static final class HangConstants {
        public static final int leftMotorChannel = 17;

        public static final double topPos = 7.72;
        public static final int servoLeftPWMId = 0;

        public static final float servoReleaseLeft = 0;
        public static final float servoHoldLeft = 0;

        public static final double hangPower = 0.3;
        public static final double toHangPos = -0.2;
        public static final double doneHangPos = 0;
        public static final int funnelRelease = 1;
    }

    // #region
    SparkMax hangMotor;
    Servo funnelRelease;
    Servo ratchetRelease;

    boolean funnelState = true; // true is engaged false is released
    // #endregion

    public Hang() {

        hangMotor = new SparkMax(HangConstants.leftMotorChannel, MotorType.kBrushless);
        funnelRelease = new Servo(HangConstants.funnelRelease);
        ratchetRelease = new Servo(HangConstants.servoLeftPWMId);

        var config = new SparkMaxConfig();
       

        hangMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        limitSwitch = new DigitalInput(0);
        
        
    }
    //Auto Hang
    private final double target = 7.72;
    

    DigitalInput limitSwitch;
    
    public Command reset(DoubleSupplier supOne) {
        return (new RunCommand(()-> {
            setRatchetRelease();
            hangMotor.set(-supOne.getAsDouble()/3);
        }, this).until(()-> {return !limitSwitch.get();}).andThen(new InstantCommand(()-> {
            hangMotor.set(0);
            hangMotor.getEncoder().setPosition(0);
        }))).finallyDo(()->{hangMotor.set(0);setRatchetHold();}
        );
    }
    public Command autoHang(DoubleSupplier supTwo) {
        return new RunCommand(()-> {
            hangMotor.set(supTwo.getAsDouble()/3);
            setRatchetHold();
        }, this)
        .finallyDo(()->hangMotor.stopMotor());
    }
  
    public Command zeroHang() {
        return new InstantCommand(
            () -> {
                this.hangMotor.getEncoder().setPosition(0);
            }
        );
    }

   

    public Command autoOut() {
        return new RunCommand(() -> {
            hangMotor.set(-HangConstants.hangPower);
        }, this).until(this::isUnhinged).andThen(new RunCommand(() -> stop(), this));
    }

    public void stop() {
        hangMotor.set(0);
    }

    public boolean isHanged() {
        return (hangMotor.getEncoder().getPosition() > HangConstants.doneHangPos);

    }

    public boolean isUnhinged() {
        return (hangMotor.getEncoder().getPosition() < HangConstants.toHangPos);
    }

    public Command hang() {
        return new RunCommand(() -> {
            this.hangMotor.set(0.5);
        });

    }

    public void manualInput(double input) {
        if (input < -0.01) {

            hangMotor.set(input + 0.01);
            setRatchetRelease();
        } else {
            setRatchetHold();
            hangMotor.set(input);

        }

    }

    public void setFunnelRelease() {
        funnelRelease.set(1);
        funnelState = false;
    }

    public void setRatchetRelease() {
        ratchetRelease.set(0.017);
    }

    public void setRatchetHold() {
        ratchetRelease.set(0.15);
    }

    public boolean getServoState() {
        if (ratchetRelease.get() == 0.017) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("limit", limitSwitch.get());
        SmartDashboard.putNumber("Hang Pos", hangMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("abosulte Pos", hangMotor.getAbsoluteEncoder().getPosition());
        if (!funnelState) {
            funnelRelease.set(1);
        } else {
            funnelRelease.set(0);
        } 
    }
}

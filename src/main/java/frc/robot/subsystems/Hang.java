package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase {
    public static final class HangConstants {
        public static final int leftMotorChannel = 17;

        public static final double topPos = 8.126;
        public static final int servoLeftPWMId = 0;

        public static final float servoReleaseLeft = 0;
        public static final float servoHoldLeft = 0;

        public static final double hangPower = 0.15;
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
        config.softLimit.forwardSoftLimit(HangConstants.topPos);

        hangMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command zeroHang() {
        return new InstantCommand(
            () -> {
                this.hangMotor.getEncoder().setPosition(0);
            }
        );
    }

    public Command autoHang() {
        return new RunCommand(() -> {
            hangMotor.set(HangConstants.hangPower);
        }, this).until(this::isHanged).andThen(new RunCommand(() -> stop(), this));
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
        SmartDashboard.putNumber("Hang Pos", hangMotor.getEncoder().getPosition());
        if (!funnelState) {
            funnelRelease.set(1);
        } else {
            funnelRelease.set(0);
        }
    }
}

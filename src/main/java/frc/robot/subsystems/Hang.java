package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hang extends SubsystemBase {

    static final class HangConstants extends frc.robot.constants.Hang {};


    SparkMax drive;
    Servo ratchet;

    public Hang() {
        drive = new SparkMax(HangConstants.driveMotorId, MotorType.kBrushless);
        ratchet = new Servo(HangConstants.servoPort);
    }

    public void setPower(double power) {
        drive.set(power);
    }

    public void setReleased(boolean released) {
        ratchet.set(released ? 0.16 : 0);
    }
}

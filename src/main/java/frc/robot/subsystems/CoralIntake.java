package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
    SparkMax motor;
    LaserCan fakeBeamBreak;

    public CoralIntake(int id) {
        motor = new SparkMax(id, MotorType.kBrushless);
        fakeBeamBreak = new LaserCan(42);
        this.setDefaultCommand(new RunCommand(() -> {
            motor.set(0);
        }, this));
    }

    public void run(double speed) {
        motor.set(speed);
    }

    public boolean coralDetected() {
        var measurement = fakeBeamBreak.getMeasurement();
        if (measurement.distance_mm < 20) {
            return true;
        } else {
            return false;
        }
    }

    public void stop() {
        motor.set(0);
    }
}

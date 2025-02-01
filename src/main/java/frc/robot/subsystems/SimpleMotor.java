package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleMotor extends SubsystemBase {
    private SparkMax motor;
    private AbsoluteEncoder encoder;
   PIDController controller;
    private double kP = 0.75;
    private double target = 0;
    public SimpleMotor(int canId) {
        motor = new SparkMax(canId, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
        controller = new PIDController(kP, 0, 0);

        controller.enableContinuousInput(0, 1);
        controller.setSetpoint(0.0);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }
    public void setTarget(float target) {
        this.target = target;
    }
    public void periodic() {
        //motor.set(-controller.calculate(getPos(), target));
        
        System.out.println(getPos());
    }
    public void setTop() {
        target = 0.057;
    }
    public void setDown() {
        target = 0.024;
    }
    public double getPos() {
        return encoder.getPosition();
    }
}

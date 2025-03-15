package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Carbon fiber algae knocker subsystem
 */
public class Bonk extends SubsystemBase{
    SparkMax bonker;
    private final double endPos = 0;
    private final double bonkPower = 0.2;
    public Bonk(){
        bonker = new SparkMax(50, MotorType.kBrushless);
        throw new IllegalAccessError("Bonk is initialized with crappy values");
    }
    public Command bonk() {
        return new RunCommand(()->bonker.set(bonkPower), this).until(()->{return bonker.getEncoder().getPosition() > endPos;}).andThen(()->bonker.set(0));
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pos", bonker.getEncoder().getPosition());
    }
    
}

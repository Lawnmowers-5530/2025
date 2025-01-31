package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class TalonFxMotor implements Motor {
    TalonFX motor;

    public TalonFxMotor(int canID) {
        motor = new TalonFX(canID);
    }

    @Override
    public void setSpeed(double speed) {
        DutyCycleOut control = new DutyCycleOut(speed);
        motor.setControl(control);
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        switch (mode) {
            case Brake:
                motor.setNeutralMode(NeutralModeValue.Brake);
                break;
            case Coast:
                motor.setNeutralMode(NeutralModeValue.Coast);
                break;
        }
    }
}

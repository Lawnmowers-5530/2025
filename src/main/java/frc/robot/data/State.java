package frc.robot.data;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

public sealed interface State
permits 
    State.Idling,
    State.Driving, 
    State.Climbing 
{

    public class DriveState {
        Vector<N2> driveVector;
        double driveRotation;
        boolean slowMode;
    }

    public record Driving(DriveState driveState) implements State {}

    public class ClimbState {
        public double speed;
    }

    public record Climbing(ClimbState climbState) implements State {}

    public class IdleState {}

    public record Idling() implements State {}
}


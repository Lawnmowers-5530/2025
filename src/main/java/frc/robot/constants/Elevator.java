package frc.robot.constants;

public class Elevator {
    public static final double level4 = 87;
    public static final double level3 = 70;
    public static final double level2 = 50;
    public static final double level1 = 30; //unsure of actual value
    public static final double intake = 5;

    public static final double kP1 = 0.0088888888;
    public static final double kI1 = 0.1;

    public static final double kS = 0.1;
    public static final double kV = 0.0; //TODO: need to fix ff vals
    public static final double kA = 0;

    public static final double maxAcceleration = 15;
    public static final double maxDeacceleration = 15;
    public static final double maxVelocity = 30;
    public static final int motor1Id = 20;
    public static final int motor2Id = 21;
    public static final int limitSwitchChannel = 1;
    public static final int ticksPerInch = 200;

    public static final int calibrationBottomBufferTicks = 10;
    public static final int calibrationTopBufferTicks = 10;
    public static final double calibrationSpeed = 0.2;
    public static final int elevatorRangeTicks = 0;

    public static final double tolerance = 2;
}

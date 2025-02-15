package frc.robot.constants;

public class Elevator {
    public static final double level4 = 0.41;
    public static final double level3 = 0.26;
    public static final double level2 = 0.16;
    public static final double level1 = 0.1; //unsure of actual value
    public static final double intake = 0.01;

    public static final double kP1 = 0.8;
    public static final double kI1 = 0.1;

    public static final double kS = 0.1;
    public static final double kV = 0.1; //TODO: need to fix ff vals
    public static final double kA = 0;

    public static final double maxAcceleration = 0.1;
    public static final double maxDeacceleration = 0.1;
    public static final double maxVelocity = 0.15;
    public static final int motor1Id = 6;
    public static final int motor2Id = 8;
    public static final int limitSwitchChannel = 1;
    public static final int ticksPerInch = 200;

    public static final int calibrationBottomBufferTicks = 10;
    public static final int calibrationTopBufferTicks = 10;
    public static final double calibrationSpeed = 0.2;
    public static final int elevatorRangeTicks = 0;
}

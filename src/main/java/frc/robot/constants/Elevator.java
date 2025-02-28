package frc.robot.constants;

public class Elevator {
    public static final double level4 = 91.5;
    public static final double level3 = 55; //unsure of actual value
    public static final double level2 = 33; //unsure of actual value
    public static final double level1 = 14; //unsure of actual value was 28
    public static final double intake = 0.2;

    public static final double minSpeed = -0.6;
    public static final double maxSpeed = 0.85;

    public static final double kP = 0.033;
    public static final double kI = 0.000;
    public static final double kD = 0.0;
    public static final double integralZone = 2;

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

    public static final double tolerance = 3;
}

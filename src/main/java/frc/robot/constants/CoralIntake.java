package frc.robot.constants;

public class CoralIntake {
    public static class Pivot {
        public static final int pivotId = 22;
        public static final int intakeId = 23;
        public static final int laserCan1Id = 42;
        public static final int laserCan2Id = 27;

        public static final double intakePower = 0.45;

        public static final double Kp = 1.65;
        public static final double tolerance = 0.06;

        //Targets (Degrees)w
        public static final double intakePos = 0.76;
        public static final double middlePos = 0.762;
        public static final double topPos = 0.63;
        public static final double bottomPos = 0.858;
        public static final double L4 = 0.72;


        //Feedforward Calculations
        public static final double length = 0;

        public static final double restistance = 1;
        public static final double mass = 2;
        public static final double gravity = 9.81;
        //Inverse
        public static final double center = 1 / 2.0;
        public static final double gearRatio = 1 / 36.0;
        public static final double kt = (105 - 1.8) / 2.6;
    }
}
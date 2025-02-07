package frc.robot.constants;

public class CoralIntake {
    public static class Pivot {
        public static final int pivotId = 0;
        public static final int intakeId = 1;
        public static final int laserCan1Id = 1;
        public static final int laserCan2Id = 2;

        public static final double intakePower = 0.3;

        public static final double Kp = 0.01;

        //Targets (Degrees)
        public static final double intakePos = 0;
        public static final double middlePos = 0;
        public static final double topPos = 0;
        public static final double bottomPos = 0;


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


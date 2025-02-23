package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class AlignToTag {
    public static final double xkPtrans = 0.17;
    public static final double xkItrans = 0;
    public static final double xkDtrans = 0;

    public static final double ykPtrans = 0.19;
    public static final double ykItrans = 0;
    public static final double ykDtrans = 0;

    public static final double kProt = 0.03;
    public static final double kIrot = 0.0;
    public static final double kDrot = 0;

    public static final Transform3d leftCameraToRobot = new Transform3d(-0.1, -0.07, 0, new Rotation3d(0, 0, 0)); //TODO prossibly swapped
    public static final Transform3d rightCameraToRobot = new Transform3d(-0.1, 0, 0, new Rotation3d(0, 0, 0));

    public static final double driveTolerance = 0.01;
    public static final double rotatationTolerance = 0.5;

    public static final boolean  useGyro = true;

}

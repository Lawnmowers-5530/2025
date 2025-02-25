package frc.robot.constants;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class AlignToTag {
    public static final double xkPtrans = 0.28;
    public static final double xkItrans = 0;
    public static final double xkDtrans = 0;

    public static final double ykPtrans = 0.33;
    public static final double ykItrans = 0;
    public static final double ykDtrans = 0;

    public static final double kProt = 0.027;
    public static final double kIrot = 0.0;
    public static final double kDrot = 0;

    public static final Transform3d leftCameraToRobot = new Transform3d(-0, 0+0.02, 0, new Rotation3d(0, 0, 0));
    public static final Transform3d rightCameraToRobot = new Transform3d(-0, 0.0208+0.02, 0, new Rotation3d(0, 0, 0));

    public static final double yDriveTolerance = 0.01;
    public static final double xDriveTolerance = 0.025;
    public static final double rotatationTolerance = 0.5;

    public static final boolean  useGyro = true;
    public static HashMap<Integer, Transform3d> tagOffsets = new HashMap<>();
    public void initializeTagOffsets() {
        //Tag Offset Code Goes Here
    }


}

package frc.robot.constants;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class AlignToTag implements Sendable {
    public static final double xkPtrans = 0.32;
    public static final double xkItrans = 0;
    public static final double xkDtrans = 0;

    public static final double ykPtrans = 0.38;
    public static final double ykItrans = 0;
    public static final double ykDtrans = 0;

    public static final double kProt = 0.027;
    public static final double kIrot = 0.0;
    public static final double kDrot = 0;

    public static Transform3d leftCameraToRobot = new Transform3d(0, 0.016, 0, new Rotation3d(0, 0, 0)); //working for aligning to right coral post
    public static Transform3d rightCameraToRobot = new Transform3d(0, 0.08, 0, new Rotation3d(0, 0, 0));
    //Reanissance Constants are 0.016 and 0.08
    @Override
     public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Camera Offsets");
        builder.addDoubleProperty("Left Offset", ()-> {return leftCameraToRobot.getY();}, (d)-> {leftCameraToRobot = new Transform3d(0,d,0,Rotation3d.kZero);});
        builder.addDoubleProperty("Right Offset", ()-> {return rightCameraToRobot.getY();}, (d)-> {rightCameraToRobot = new Transform3d(0,d,0,Rotation3d.kZero);});
       
    }

    public static final double yDriveTolerance = 0.005;
    public static final double xDriveTolerance = 0.025;
    public static final double rotatationTolerance = 0.25;

    public static final boolean  useGyro = true; 
    public static HashMap<Integer, Transform3d> tagOffsets = new HashMap<>();
    /*public void initializeTagOffsets() {
        tagOffsets = new HashMap<>();
        tagOffsets.put(6, new Transform3d(0, 0, 0, new Rotation3d()));
        tagOffsets.put(7, new Transform3d(0, 0, 0, new Rotation3d()));
        tagOffsets.put(8, new Transform3d(0, 0, 0, new Rotation3d()));
        tagOffsets.put(9, new Transform3d(0, 0, 0, new Rotation3d()));
        tagOffsets.put(10, new Transform3d(0, 0, 0, new Rotation3d()));
        tagOffsets.put(11, new Transform3d(0, 0, 0, new Rotation3d()));
        tagOffsets.put(18, new Transform3d(0, 0, 0, new Rotation3d()));
        tagOffsets.put(19, new Transform3d(0, 0, 0, new Rotation3d())); 
        tagOffsets.put(20, new Transform3d(0, 0, 0, new Rotation3d()));
        tagOffsets.put(21, new Transform3d(0, 0, 0, new Rotation3d()));
        tagOffsets.put(22, new Transform3d(0, 0, 0, new Rotation3d()));
        tagOffsets.put(17, new Transform3d(0, 0, 0, new Rotation3d()));
    }*/


}

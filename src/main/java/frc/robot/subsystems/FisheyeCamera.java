package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FisheyeCameraConstants;

public class FisheyeCamera extends SubsystemBase {
    UsbCamera camera;
    Mat image;
    CvSink sink;
    CvSource output;
    
    public FisheyeCamera(FisheyeCameraConstants c) {
        camera = CameraServer.startAutomaticCapture(c.name, c.port);

        camera.setVideoMode(PixelFormat.kMJPEG, c.width, c.height, c.fps);
        camera.setExposureManual(32);
        sink = CameraServer.getVideo(camera);
        output = CameraServer.putVideo(c.name, c.width, c.height);
        image = new Mat();
        CommandScheduler.getInstance().registerSubsystem(this);
       
    }

    @Override
    public void periodic() {
        if (!camera.isConnected()) {
            System.out.println("no cam disconnected");
            return;
        }
        long res = sink.grabFrame(image);
        if (res == 0) {
            System.out.println("no cam");
            return;
        }
        output.putFrame(image);
    }
    /*
     * 
     */
    

    
}
package frc.robot.subsystems;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import frc.robot.Constants.VisionConstants;

public class VisionManager extends SubsystemBase {
    private static final VisionManager instance = new VisionManager();

    public static VisionManager getInstance() {
        return instance;
    }

    UsbCamera usbCamera;
    CvSource outputStream;
    CvSink cvSink;

    public VisionManager() {
        usbCamera = CameraServer.startAutomaticCapture();
        usbCamera.setResolution(VisionConstants.UsbCameraResolution[0], VisionConstants.UsbCameraResolution[1]);
        outputStream = CameraServer.putVideo("ElevatorCAM", VisionConstants.UsbCameraResolution[0], VisionConstants.UsbCameraResolution[1]);
       
    }

    public boolean isCone() {
        Mat img = new Mat();
        if (cvSink.grabFrame(img) == 0) {
            outputStream.notifyError(cvSink.getError());
            return false;
        };
        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV);
        double[] centerColor = img.get(VisionConstants.UsbCameraResolution[1]/2, VisionConstants.UsbCameraResolution[0]/2);
        SmartDashboard.putNumberArray("centerColor", centerColor);
        return !(centerColor[0] < VisionConstants.kYellowLowThreshold[0] || centerColor[0] > VisionConstants.kYellowHighThreshold[0] ||
                centerColor[1] < VisionConstants.kYellowLowThreshold[1] || centerColor[1] > VisionConstants.kYellowHighThreshold[1] ||
                centerColor[2] < VisionConstants.kYellowLowThreshold[2] || centerColor[2] > VisionConstants.kYellowHighThreshold[2]
        );
    }

    boolean isFirstConnected = true;
    @Override
    public void periodic() {
        if (usbCamera.isConnected()) cvSink = CameraServer.getVideo(usbCamera);
    }
}


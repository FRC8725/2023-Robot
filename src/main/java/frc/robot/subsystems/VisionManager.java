package frc.robot.subsystems;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.FieldConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionManager extends SubsystemBase {
    private static final VisionManager instance = new VisionManager();

    public static VisionManager getInstance() {
        return instance;
    }

    PhotonCamera camera = new PhotonCamera("OV5647");

    UsbCamera usbCamera;
    CvSource outputStream;
    CvSink cvSink;
    PhotonPipelineResult result = new PhotonPipelineResult();
    // Pair<PhotonCamera, Transform3d> campair = new Pair<PhotonCamera, Transform3d>(camera, VisionConstants.Photon2Robot);
    PhotonPoseEstimator estimator = new PhotonPoseEstimator(FieldConstants.aprilTagField, PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, VisionConstants.Robot2Photon);

    public VisionManager() {
        usbCamera = CameraServer.startAutomaticCapture(0);
        usbCamera.setResolution(VisionConstants.UsbCameraResolution[0], VisionConstants.UsbCameraResolution[1]);
        outputStream = CameraServer.putVideo("ElevatorCAM", VisionConstants.UsbCameraResolution[0], VisionConstants.UsbCameraResolution[1]);
        setDriverMode(false);
    }

    public Transform3d getAprilTagRelative() {
        camera.setPipelineIndex(1);
        PhotonTrackedTarget target;
        Transform3d bestCameraToTarget = new Transform3d();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            bestCameraToTarget = target.getBestCameraToTarget();
        }
        return bestCameraToTarget;
    }

    public double getReflectiveTapeRelativeYawRads() {
        camera.setPipelineIndex(0);
        PhotonTrackedTarget target;
        double angle = 0;
        if (result.hasTargets()) {
            target = result.getBestTarget();
            angle = -Units.degreesToRadians(target.getYaw());
        }
        return angle;
    }

    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        camera.setPipelineIndex(1);
        estimator.setReferencePose(prevEstimatedRobotPose);

        double currentTime = Timer.getFPGATimestamp();
        Optional<EstimatedRobotPose> estimatorResult = estimator.update();
        if (estimatorResult.isPresent()) {
            return new Pair<Pose2d, Double>(
                    estimatorResult.get().estimatedPose.toPose2d(), currentTime - estimatorResult.get().timestampSeconds);
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }

    public boolean hasTarget() {
        return result.hasTargets();
    }

    public void setDriverMode(boolean driverMode) {
        camera.setDriverMode(driverMode);
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
        // inRange(HSV, hThreshold, lThreshold)
        return false;
//        return !(centerColor[0] < VisionConstants.kYellowLowThreshold[0] || centerColor[0] > VisionConstants.kYellowHighThreshold[0] ||
//                centerColor[1] < VisionConstants.kYellowLowThreshold[1] || centerColor[1] > VisionConstants.kYellowHighThreshold[1] ||
//                centerColor[2] < VisionConstants.kYellowLowThreshold[2] || centerColor[2] > VisionConstants.kYellowHighThreshold[2]
//        );
    }

    public double getConeAngleRads() {
        Mat src = new Mat();
        if (cvSink.grabFrame(src) == 0) {
            outputStream.notifyError(cvSink.getError());
            return 0;
        };
        Mat blurred = new Mat(src.rows(), src.rows(), src.type());
        Imgproc.GaussianBlur(src, blurred, new Size(15, 15), 0);
        Mat img_HSV = new Mat(src.rows(), src.rows(), src.type());
        Imgproc.cvtColor(img_HSV, blurred, Imgproc.COLOR_BGR2HSV);
        Mat img_thresh = new Mat(src.rows(), src.rows(), src.type());
        Core.inRange(img_HSV, VisionConstants.kYellowLowThreshold, VisionConstants.kYellowHighThreshold, img_thresh);
        Mat img_dilate = new Mat(src.rows(), src.rows(), src.type());
        Imgproc.dilate(img_thresh, img_dilate, new Mat(5, 5, src.type()));
        Mat img_edges = new Mat(src.rows(), src.rows(), src.type());
        Imgproc.Canny(img_dilate, img_edges, 30, 180);
        List<MatOfPoint> cnts = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(img_edges, cnts, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint cnt : cnts) {
            MatOfPoint2f cnt2f = new MatOfPoint2f();
            MatOfPoint2f approx2f = new MatOfPoint2f();
            MatOfPoint approx = new MatOfPoint();
            cnt.convertTo(cnt2f, CvType.CV_32FC2);
            Imgproc.approxPolyDP(cnt2f, approx2f, 2, true);
            approx2f.convertTo(approx, CvType.CV_32S);
            if (approx.size().height == 3) {
                double[] a = approx.get(0, 0);
                double[] b = approx.get(1, 0);
                double[] c = approx.get(2, 0);
                double l1 = norml2(a, b);
                double l2 = norml2(a, b);
                double l3 = norml2(a, b);
            }
        }
        return 0;
    }

    private double norml2(double[] a, double[] b) {
        return Math.sqrt(Math.pow(a[0], 2) + Math.pow(b[0], 2)) + Math.sqrt(Math.pow(a[1], 2) + Math.pow(b[1], 2));
    }

    boolean isFirstConnected = true;
    @Override
    public void periodic() {
        cvSink = CameraServer.getVideo(usbCamera);
        if (!camera.isConnected()) return;
        if (isFirstConnected) {
            camera.setDriverMode(true);
            isFirstConnected = false;
        }
        result = camera.getLatestResult();
    }
}


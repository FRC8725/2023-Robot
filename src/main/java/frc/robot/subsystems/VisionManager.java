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

    public double getConeAngleRads() {
        Mat src = new Mat();
        if (cvSink.grabFrame(src) == 0) {
            outputStream.notifyError(cvSink.getError());
            return 0;
        };
        Mat img = new Mat(src.rows(), src.rows(), src.type());
        Imgproc.GaussianBlur(src, img, new Size(11, 11), 0);
        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV);
        Core.inRange(img, VisionConstants.kYellowLowThreshold, VisionConstants.kYellowHighThreshold, img);
        Imgproc.dilate(img, img, new Mat(3, 3, CvType.CV_32S), new Point(-1, -1), 2);
        Imgproc.erode(img, img, new Mat(3, 3, CvType.CV_32S), new Point(-1, -1), 2);

        List<MatOfPoint> cnts = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(img, cnts, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxVal = 0;
        int maxIdx = 0;
        for (int cntIdx = 0; cntIdx < cnts.size(); cntIdx++) {
            double counterArea = Imgproc.contourArea(cnts.get(cntIdx));
            if (maxVal < counterArea) {
                maxVal = counterArea;
                maxIdx = cntIdx;
            }
        }

        if (maxVal == 0) return 0;

        MatOfPoint2f cnt2f = new MatOfPoint2f(cnts.get(maxIdx).toArray());
        double peri = Imgproc.arcLength(cnt2f, true);
        MatOfPoint2f approx2f = new MatOfPoint2f();
        MatOfPoint approx = new MatOfPoint();

        Imgproc.approxPolyDP(cnt2f, approx2f, 0.02 * peri, true);
        Point[] pointsArray = approx.toArray();
        Mat points = new Mat(pointsArray.length, 1, CvType.CV_32FC2);
        for (int i = 0; i < pointsArray.length; i++) {
            Point point = pointsArray[i];
            points.put(i, 0, point.x, point.y);
        }
        Mat triangle = new Mat();
        Imgproc.minEnclosingTriangle(points, triangle);

        double[] p1 = triangle.get(0, 0);
        double[] p2 = triangle.get(1, 0);
        double[] p3 = triangle.get(2, 0);

        double p1p2 = norml2(p1, p2);
        double p2p3 = norml2(p2, p3);
        double p1p3 = norml2(p1, p3);

        double angle = 0;

        if (p1p2 < p2p3 && p1p2 < p1p3) {
            double[] mid = {(p1[0]+p2[0])/2, (p1[1]+p2[1])/2};
            angle = Math.atan((p3[1]-mid[1])/(p3[0]-mid[0]));
        } else if (p2p3 < p1p2 && p2p3 < p1p3) {
            double[] mid = {(p2[0]+p3[0])/2, (p2[1]+p3[1])/2};
            angle = Math.atan((p1[1]-mid[1])/(p1[0]-mid[0]));
        } else if (p1p3 < p2p3 && p1p3 < p1p2) {
            double[] mid = {(p1[0]+p3[0])/2, (p1[1]+p3[1])/2};
            angle = Math.atan((p2[1]-mid[1])/(p2[0]-mid[0]));
        }
        return angle;
    }

    private double norml2(double[] a, double[] b) {
        return Math.sqrt(Math.pow(a[0] - b[0], 2)) + Math.sqrt(Math.pow(a[1] - b[1], 2));
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


package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Arm.Elbow;
import frc.robot.subsystems.Arm.Winch;
import frc.robot.subsystems.Arm.Wrist;

public class ArmSubsystem extends SubsystemBase {

    private final static ArmSubsystem INSTANCE = new ArmSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }

    private final Elbow elbow;
    private final Winch winch;
    private final Wrist wrist;
    private final Pneumatics pneumatics;

    boolean isResetting;
    double lastX = ElevatorConstants.kForearmLength;
    double lastY = ElevatorConstants.kUpperArmLength;

    boolean isHorizontal = true;
    boolean isTransporting = true;

    private ArmSubsystem() {
        elbow = Elbow.getInstance();
        winch = Winch.getInstance();
        wrist = Wrist.getInstance();
        pneumatics = Pneumatics.getInstance();
        reset();
    }

    @Override
    public void periodic() {
        wrist.setWristSetpoint(elbow.getAbsoluteEncoderRad() - Math.PI/2 + winch.getAbsoluteEncoderRad() + (isHorizontal? 0: -Math.PI/2) + (pneumatics.getGripperStatus()? 0: Units.degreesToRadians(10)) + (isTransporting? Units.degreesToRadians(70): 0));
        SmartDashboard.putBoolean("atArmSetpoint", atSetpoint());
    }

    public void reset() {
//        elevator.setSetpoint(ElevatorConstants.kMinElevatorHeight);
//        arm.setSetpoint(ElevatorConstants.kMinArmHeight);
        elbow.resetEncoder();
        winch.resetEncoder();
        wrist.resetEncoder();

        elbow.setSetpoint(ElevatorConstants.kMaxElbowAngle);
        winch.setSetpoint(ElevatorConstants.kMinWinchAngle);
        lastX = ElevatorConstants.kForearmLength;
        lastY = ElevatorConstants.kUpperArmLength;
        isResetting = true;
        isTransporting = true;
    }

    private double LawOfCosinesTheta(double a, double b, double c) {
        double result = (Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c, 2)) / (2 * a * b);
        if (Math.abs(result) > 1) return -1;
        return Math.acos(result);
    }

    public void setSetpoint(double xAxis, double yAxis) {
        double distanceSquared = Math.pow(xAxis, 2) + Math.pow(yAxis, 2);
        double distance = Math.sqrt(distanceSquared);

        if (distance >= ElevatorConstants.kForearmLength + ElevatorConstants.kUpperArmLength) return;
        else if (distance <= ElevatorConstants.kUpperArmLength - ElevatorConstants.kForearmLength) return;
        else if (xAxis > ElevatorConstants.kMaxXAxis || xAxis < ElevatorConstants.kMinXAxis) return;
        else if (yAxis > ElevatorConstants.kMaxYAxis || yAxis < ElevatorConstants.kMinYAxis) return;

        SmartDashboard.putNumber("setX", xAxis);
        SmartDashboard.putNumber("setY", yAxis);
        SmartDashboard.putNumber("lastX", lastX);
        SmartDashboard.putNumber("lastY", lastY);

        double thetaElbow, thetaWinch;
        double l1 = ElevatorConstants.kUpperArmLength;
        double l2 = ElevatorConstants.kForearmLength;
        double l3 = distance;
        double theta1 = LawOfCosinesTheta(l1, l2, l3);
        thetaElbow = Math.PI - theta1;
        double theta2 = LawOfCosinesTheta(l1, l3, l2);
        if (Math.abs(yAxis / l3) > 1) return;
        thetaWinch = Math.acos(yAxis / l3) * (xAxis>0? 1: -1) - theta2;

        // Some insurance for the massive arm
        if (theta1 == -1 || theta2 == -1) return;
        else if (thetaElbow <  ElevatorConstants.kMinElbowAngle || thetaElbow > ElevatorConstants.kMaxElbowAngle) return;
        else if (thetaWinch <  ElevatorConstants.kMinWinchAngle || thetaWinch > ElevatorConstants.kMaxWinchAngle) return;
        lastX = xAxis;
        lastY = yAxis;
        elbow.setSetpoint(thetaElbow);
        winch.setSetpoint(thetaWinch);
    }

    public boolean atSetpoint() {
        SmartDashboard.putBoolean("atElbowSetpoint", elbow.atSetpoint());
        SmartDashboard.putBoolean("atWinchSetpoint", winch.atSetpoint());
        return elbow.atSetpoint() && winch.atSetpoint();
    }


    public void setSpeed(double spdX, double spdY) {
        if (!atSetpoint() && isResetting) return;
        isResetting = false;
        Translation2d vectorUpperArm = new Translation2d(ElevatorConstants.kUpperArmLength, Rotation2d.fromRadians(winch.getAbsoluteEncoderRad() + Math.PI/2));
        Translation2d vectorForearm = new Translation2d(ElevatorConstants.kForearmLength, Rotation2d.fromRadians(winch.getAbsoluteEncoderRad() + elbow.getAbsoluteEncoderRad() + Math.PI/2));
        Translation2d point = vectorUpperArm.plus(vectorForearm);
        SmartDashboard.putNumber("xAxis", -point.getX());
        SmartDashboard.putNumber("yAxis", point.getY());
        if (spdX == 0 && spdY == 0) return;
        if (spdX != 0) lastX = -point.getX();
        if (spdY != 0) lastY = point.getY();
        setSetpoint(lastX+spdX*ElevatorConstants.xSpdConvertFactor, lastY+spdY*ElevatorConstants.ySpdConvertFactor);
    }
//
//    public void setArmSpeed(double speed) {
//        if(speed == 0) return;
//        elbow.setSetpoint(elbow.getEncoder() + speed/ElevatorConstants.kPElbow);
//    }
//
    public void setHorizontal(boolean isHorizontal) {
        this.isHorizontal = isHorizontal;
    }

    public void setTransporting(boolean isTransporting) {
        this.isTransporting = isTransporting;
    }

    public void stop() {
        elbow.stop();
        winch.stop();
    }
}


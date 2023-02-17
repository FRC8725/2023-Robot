package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
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

    double lastX, lastY;

    boolean isResetting = true;
    boolean isHorizontal = true;
    boolean isTransporting = true;
    boolean isPlacing = false;

    private ArmSubsystem() {
        Timer.delay(1.5);
        elbow = Elbow.getInstance();
        winch = Winch.getInstance();
        wrist = Wrist.getInstance();
        reset();
    }

    @Override
    public void periodic() {
        if (isResetting) {
            if (atSetpoint()) isResetting = false;
            else if (winch.atSetpoint()) elbow.setSetpoint(ArmConstants.kMaxElbowAngle);
        }
//        isTransporting = false;
        wrist.setWristSetpoint(elbow.getAbsoluteEncoderRad() - Math.PI/2 + winch.getAbsoluteEncoderRad() + (isHorizontal? 0: -Math.PI/2) + (isPlacing? Units.degreesToRadians(10) : 0) + (isTransporting? Units.degreesToRadians(70): 0));
//        wrist.setWristSetpoint(0);
        SmartDashboard.putBoolean("atArmSetpoint", atSetpoint());
        elbow.calculate();
        winch.calculate();
        wrist.calculate();
    }

    public void reset() {
//        elevator.setSetpoint(ElevatorConstants.kMinElevatorHeight);
//        arm.setSetpoint(ElevatorConstants.kMinArmHeight);
        winch.setSetpoint(ArmConstants.kMinWinchAngle);
        elbow.setSetpoint((3 * ArmConstants.kMaxElbowAngle + 1 * ArmConstants.kMinElbowAngle) / 4);
//        elbow.setSetpoint(ArmConstants.kMaxElbowAngle); // Move to periodic()
        Translation2d vectorUpperArm = new Translation2d(ArmConstants.kUpperArmLength, Rotation2d.fromRadians(ArmConstants.kMinWinchAngle + Math.PI/2));
        Translation2d vectorForearm = new Translation2d(ArmConstants.kForearmLength, Rotation2d.fromRadians(ArmConstants.kMinWinchAngle + ArmConstants.kMaxElbowAngle + Math.PI/2));
        Translation2d point = vectorUpperArm.plus(vectorForearm);
        lastX = -point.getX();
        lastY = point.getY();
        isResetting = true;
        isTransporting = true;
        isHorizontal = true;
        isPlacing = false;
    }

    private double LawOfCosinesTheta(double a, double b, double c) {
        double result = (Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c, 2)) / (2 * a * b);
        if (Math.abs(result) > 1) return -1;
        return Math.acos(result);
    }

    public void setSetpoint(double xAxis, double yAxis) {
        double distanceSquared = Math.pow(xAxis, 2) + Math.pow(yAxis, 2);
        double distance = Math.sqrt(distanceSquared);

        if (distance >= ArmConstants.kForearmLength + ArmConstants.kUpperArmLength) return;
        else if (distance <= ArmConstants.kUpperArmLength - ArmConstants.kForearmLength) return;
        else if (xAxis > ArmConstants.kMaxXAxis || xAxis < ArmConstants.kMinXAxis) return;
        else if (yAxis > ArmConstants.kMaxYAxis || yAxis < ArmConstants.kMinYAxis) return;

        SmartDashboard.putNumber("setX", xAxis);
        SmartDashboard.putNumber("setY", yAxis);
        SmartDashboard.putNumber("lastX", lastX);
        SmartDashboard.putNumber("lastY", lastY);

        double thetaElbow, thetaWinch;
        double l1 = ArmConstants.kUpperArmLength;
        double l2 = ArmConstants.kForearmLength;
        double l3 = distance;
        double theta1 = LawOfCosinesTheta(l1, l2, l3);
        thetaElbow = Math.PI - theta1;
        double theta2 = LawOfCosinesTheta(l1, l3, l2);
        if (Math.abs(yAxis / l3) > 1) return;
        thetaWinch = Math.acos(yAxis / l3) * (xAxis>0? 1: -1) - theta2;

        // Some insurance for the massive arm
        if (theta1 == -1 || theta2 == -1) return;
        else if (thetaElbow <  ArmConstants.kMinElbowAngle || thetaElbow > ArmConstants.kMaxElbowAngle) return;
        else if (thetaWinch <  ArmConstants.kMinWinchAngle || thetaWinch > ArmConstants.kMaxWinchAngle) return;
        lastX = xAxis;
        lastY = yAxis;
        isResetting = false;
        elbow.setSetpoint(thetaElbow);
        winch.setSetpoint(thetaWinch);
    }

    public boolean atSetpoint() {
        SmartDashboard.putBoolean("atElbowSetpoint", elbow.atSetpoint());
        SmartDashboard.putBoolean("atWinchSetpoint", winch.atSetpoint());
        return elbow.atSetpoint() && winch.atSetpoint();
    }


    public void setSpeed(double spdX, double spdY) {
        if (isResetting) return;
        Translation2d vectorUpperArm = new Translation2d(ArmConstants.kUpperArmLength, Rotation2d.fromRadians(winch.getAbsoluteEncoderRad() + Math.PI/2));
        Translation2d vectorForearm = new Translation2d(ArmConstants.kForearmLength, Rotation2d.fromRadians(winch.getAbsoluteEncoderRad() + elbow.getAbsoluteEncoderRad() + Math.PI/2));
        Translation2d point = vectorUpperArm.plus(vectorForearm);
        SmartDashboard.putNumber("xAxis", -point.getX());
        SmartDashboard.putNumber("yAxis", point.getY());
        if (spdX == 0 && spdY == 0) return;
        if (spdX != 0) lastX = -point.getX();
        if (spdY != 0) lastY = point.getY();
        setSetpoint(lastX+spdX* ArmConstants.xSpdConvertFactor, lastY+spdY* ArmConstants.ySpdConvertFactor);
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

    public void setPlacing(boolean isPlacing) {
        this.isPlacing = isPlacing;
    }

    public void stop() {
        elbow.stop();
        winch.stop();
    }
}


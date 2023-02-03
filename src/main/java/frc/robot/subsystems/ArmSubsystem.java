package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator.Elbow;
import frc.robot.subsystems.Elevator.Winch;

public class ArmSubsystem extends SubsystemBase {

    private final static ArmSubsystem INSTANCE = new ArmSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }

    private final Elbow elbow;
    private final Winch winch;

    boolean isResetting;

    private ArmSubsystem() {
        elbow = Elbow.getInstance();
        winch = Winch.getInstance();
        reset();
        Timer.delay(2);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("atArmSetpoint", atSetpoint());
    }

    public void reset() {
//        elevator.setSetpoint(ElevatorConstants.kMinElevatorHeight);
//        arm.setSetpoint(ElevatorConstants.kMinArmHeight);
        elbow.resetEncoder();
        winch.resetEncoder();
        elbow.setSetpoint(Math.PI/2);
        winch.setSetpoint(0);
        isResetting = true;
    }

    private double LawOfCosinesTheta(double a, double b, double c) {
        double result = (Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c, 2)) / (2 * a * b);
        if (Math.abs(result) > 1) return -1;
        return Math.acos(result);
    }

    public void setSetpoint(double xAxis, double yAxis) {
        double distanceSquared = Math.pow(xAxis, 2) + Math.pow(yAxis, 2);
        double distance = Math.sqrt(distanceSquared);
        SmartDashboard.putNumber("Distance", distance);
        if (distance >= ElevatorConstants.kForearmLength + ElevatorConstants.kUpperArmLength) return;
        else if (distance <= ElevatorConstants.kUpperArmLength - ElevatorConstants.kForearmLength) return;

        SmartDashboard.putNumber("setX", xAxis);
        SmartDashboard.putNumber("setY", yAxis);

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
        elbow.setSetpoint(thetaElbow);
        winch.setSetpoint(thetaWinch);
    }

    public boolean atSetpoint() {
        return elbow.atSetpoint() && winch.atSetpoint();
    }

    public void setSpeed(double spdX, double spdY) {
        if (!atSetpoint() && isResetting) return;
        isResetting = false;
        Translation2d vectorUpperArm = new Translation2d(ElevatorConstants.kUpperArmLength, Rotation2d.fromRadians(winch.getAbsoluteEncoderRad() + Math.PI/2));
        Translation2d vectorForearm = new Translation2d(ElevatorConstants.kForearmLength, Rotation2d.fromRadians(winch.getAbsoluteEncoderRad() + elbow.getAbsoluteEncoderRad() + Math.PI/2));
        Translation2d point = vectorUpperArm.plus(vectorForearm);
        setSetpoint(-point.getX()+spdX*ElevatorConstants.xSpdConvertFactor, point.getY()+spdY*ElevatorConstants.ySpdConvertFactor);
        SmartDashboard.putNumber("xAxis", -point.getX());
        SmartDashboard.putNumber("yAxis", point.getY());
    }
//
//    public void setArmSpeed(double speed) {
//        if(speed == 0) return;
//        elbow.setSetpoint(elbow.getEncoder() + speed/ElevatorConstants.kPElbow);
//    }
//

    public void stop() {
        elbow.stop();
        winch.stop();
    }
}


package frc.robot.subsystems;


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

    private ArmSubsystem() {
        elbow = Elbow.getInstance();
        winch = Winch.getInstance();
        reset();
    }

    @Override
    public void periodic() {}

    public void reset() {
//        elevator.setSetpoint(ElevatorConstants.kMinElevatorHeight);
//        arm.setSetpoint(ElevatorConstants.kMinArmHeight);
        elbow.resetEncoder();
        winch.resetEncoder();
        elbow.setSetpoint(Math.PI);
        winch.setSetpoint(0);
    }

    private double LawOfCosinesTheta(double a, double b, double c) {
        double result = (Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c, 2)) / (2 * a * b);
        return Math.acos(result);
    }

    public void setSetpoint(double xAxis, double yAxis) {

        double distanceSquared = Math.pow(xAxis, 2) + Math.pow(yAxis, 2);
        if (distanceSquared >= Math.pow(ElevatorConstants.kForearmLength + ElevatorConstants.kUpperArmLength, 2)) return;

        double thetaElbow, thetaWinch;
        double l1 = ElevatorConstants.kUpperArmLength;
        double l2 = ElevatorConstants.kForearmLength;
        double l3 = Math.sqrt(distanceSquared);
        double theta1 = LawOfCosinesTheta(l1, l2, l3);
        thetaElbow = Math.PI - theta1;
        double theta2 = LawOfCosinesTheta(l1, l3, l2);
        double theta3 = LawOfCosinesTheta(Math.abs(xAxis), l3, Math.abs(yAxis));
        thetaWinch = Math.PI/2-theta3-(xAxis>0? 1: -1 * theta2);
        if (thetaWinch <  ElevatorConstants.kMinWinchAngle || thetaWinch > ElevatorConstants.kMaxWinchAngle) return;
        if (thetaElbow <  ElevatorConstants.kMinElbowAngle || thetaElbow > ElevatorConstants.kMaxElbowAngle) return;
        elbow.setSetpoint(thetaElbow);
        winch.setSetpoint(thetaWinch);
    }

    public boolean atSetpoint() {
        return elbow.atSetpoint() && winch.atSetpoint();
    }

    public void setSpeed(double spdX, double spdY) {
        double l1 = ElevatorConstants.kUpperArmLength;
        double l2 = ElevatorConstants.kForearmLength;
        double phi = winch.getEncoder();
        double theta = elbow.getEncoder();
        double xAxis = Math.sin(phi+theta)*l2 + Math.sin(phi)*l1;
        double yAxis = Math.cos(phi+theta)*l2 + Math.cos(phi)*l1;
        setSetpoint(xAxis+spdX*ElevatorConstants.xSpdConvertFactor, yAxis+spdY*ElevatorConstants.ySpdConvertFactor);
        SmartDashboard.putNumber("Distance", xAxis);
        SmartDashboard.putNumber("Height", yAxis);
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


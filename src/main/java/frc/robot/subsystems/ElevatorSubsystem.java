package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator.Elbow;
import frc.robot.subsystems.Elevator.Winch;

public class ElevatorSubsystem extends SubsystemBase {

    private final static ElevatorSubsystem INSTANCE = new ElevatorSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static ElevatorSubsystem getInstance() {
        return INSTANCE;
    }

    private final Elbow elbow;
    private final Winch winch;

    private ElevatorSubsystem() {
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

    public void setSetpoint(double distance, double height) {

        if (Math.pow(distance, 2) + Math.pow(height, 2) > Math.pow(ElevatorConstants.kForearmLength + ElevatorConstants.kUpperArmLength, 2)) return;

        double thetaElbow, thetaWinch;
        double l1 = ElevatorConstants.kUpperArmLength;
        double l2 = ElevatorConstants.kForearmLength;
        double l3 = Math.sqrt(Math.pow(distance, 2)+Math.pow(height, 2));
        double theta1 = LawOfCosinesTheta(l1, l2, l3);
        thetaElbow = Math.PI - theta1;
        double theta2 = LawOfCosinesTheta(l1, l3, l2);
        double theta3 = LawOfCosinesTheta(Math.abs(distance), l3, Math.abs(height));
        thetaWinch = Math.PI/2-theta3-theta2;
        elbow.setSetpoint(thetaElbow*(distance>0? 1: -1));
        winch.setSetpoint(thetaWinch*(distance>0? 1: -1));
    }

    public void setSpeed(double spdX, double spdY) {
        double l1 = ElevatorConstants.kUpperArmLength;
        double l2 = ElevatorConstants.kForearmLength;
        double phi = winch.getEncoder();
        double theta = elbow.getEncoder();
        double distance = Math.sin(phi+theta)*l2 + Math.sin(phi)*l1;
        double height = Math.cos(phi+theta)*l2 + Math.cos(phi)*l1;
        setSetpoint(distance+spdX*ElevatorConstants.xSpdConvertFactor, height+spdY*ElevatorConstants.ySpdConvertFactor);
        SmartDashboard.putNumber("Distance", distance);
        SmartDashboard.putNumber("Height", height);
    }

//    public void setWinchSetpoint(double setpoint) {
//        winch.setSetpoint(setpoint);
//    }
//
//    public void setArmSpeed(double speed) {
//        if(speed == 0) return;
//        elbow.setSetpoint(elbow.getEncoder() + speed/ElevatorConstants.kPElbow);
//    }
//
//    public void setWinchSpeed(double speed) {
//        if(speed == 0) return;
//        winch.setSetpoint(winch.getEncoder() + speed/ElevatorConstants.kPWinch);
//    }

    public void stop() {
        elbow.stop();
        winch.stop();
    }
}


package frc.robot.subsystems;


import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm.Elbow;
import frc.robot.subsystems.Arm.Winch;
import frc.robot.subsystems.Arm.Wrist;

public class ArmSubsystem extends SubsystemBase {
    private static final ArmSubsystem INSTANCE = new ArmSubsystem();
    private final Elbow elbow;
    private final Winch winch;
    private final Wrist wrist;
    private double lastX;
    private double lastY;
    private boolean isResetting = true;
    private boolean isHorizontal = true;
    private boolean isTransporting = true;
    private boolean isPlacing = false;
    private boolean isElbowLocked = false;
    private double wristStage = 0;
    private double desiredElbowAngle = 0;
    private double desiredWinchAngle = 0;

//    LinearFilter filter = LinearFilter.singlePoleIIR(0.2, 0.02);

    private ArmSubsystem() {
        Timer.delay(2);
        elbow = Elbow.getInstance();
        winch = Winch.getInstance();
        wrist = Wrist.getInstance();
        winch.setSetpoint(ArmConstants.INITIAL_WINCH_ANGLE);
        elbow.setSetpoint(ArmConstants.INITIAL_ELBOW_ANGLE);
        desiredElbowAngle = ArmConstants.INITIAL_ELBOW_ANGLE;
        desiredWinchAngle = ArmConstants.INITIAL_WINCH_ANGLE;
    }

    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }

    @Override
    public void periodic() {
//        isTransporting = false;
        var horizontalFunction = isHorizontal? 0: Units.degreesToRadians(40);
        var placingFunction = isPlacing? Units.degreesToRadians(30): 0;
        var transportingFunction = isTransporting? Units.degreesToRadians(90): 0;
        var teleopAdjFunction = Units.degreesToRadians(10) * wristStage;
        var offset = horizontalFunction + placingFunction + transportingFunction + teleopAdjFunction;
        var wristSetpoint = elbow.getSetpoint() - Math.PI / 2 + winch.getSetpoint() + offset;
        wrist.setWristSetpoint(wristSetpoint);
//        wrist.setWristSetpoint(0);
        wrist.calculate();

        if (isResetting) {
            if (winch.atSetpoint()) {
                winch.setSetpoint(ArmConstants.INITIAL_WINCH_ANGLE);
                elbow.setSetpoint(ArmConstants.INITIAL_ELBOW_ANGLE);
                isElbowLocked = false;
            }
            if (atSetpoint()) {
                var armPosition = getArmPosition();
                lastX = armPosition.getFirst();
                lastY = armPosition.getSecond();
                winch.setSetpoint(ArmConstants.INITIAL_WINCH_ANGLE);
                elbow.setSetpoint(ArmConstants.INITIAL_ELBOW_ANGLE);
                isElbowLocked = false;
                isResetting = false;
                desiredElbowAngle = ArmConstants.INITIAL_ELBOW_ANGLE;
                desiredWinchAngle = ArmConstants.INITIAL_WINCH_ANGLE;
            }
        }
        if (isElbowLocked) {
            elbow.setSetpoint(desiredElbowAngle - (winch.getAbsoluteEncoderRad() - desiredWinchAngle));
        }
        SmartDashboard.putBoolean("atArmSetpoint", atSetpoint());
        elbow.calculate();
        winch.calculate();
    }

    public void reset() {
//        elevator.setSetpoint(ElevatorConstants.kMinElevatorHeight);
//        arm.setSetpoint(ElevatorConstants.kMinArmHeight);
//        elbow.setSetpoint(ArmConstants.MAX_ELBOW_ANGLE); // Move to periodic()
//        Translation2d vectorUpperArm = new Translation2d(ArmConstants.UPPER_ARM_LENGTH, Rotation2d.fromRadians(ArmConstants.MIN_WINCH_ANGLE + Math.PI/2));
//        Translation2d vectorForearm = new Translation2d(ArmConstants.FOREARM_LENGTH, Rotation2d.fromRadians(ArmConstants.MIN_WINCH_ANGLE + ArmConstants.MAX_ELBOW_ANGLE + Math.PI/2));
//        Translation2d point = vectorUpperArm.plus(vectorForearm);
        elbow.setSetpoint(ArmConstants.INITIAL_ELBOW_ANGLE);
        winch.setSetpoint(ArmConstants.INITIAL_WINCH_ANGLE);
        var armPosition = getArmPosition();
        lastX = armPosition.getFirst();
        lastY = armPosition.getSecond();
        if(!atSetpoint()) {
            winch.setSetpoint(ArmConstants.MIN_WINCH_ANGLE);
            isElbowLocked = true;
            if (elbow.getAbsoluteEncoderRad() < Units.degreesToRadians(90)) {
                desiredElbowAngle = Units.degreesToRadians(90);
                desiredWinchAngle = 0;
            }
        }
        isResetting = true;
        isTransporting = true;
        isHorizontal = true;
        isPlacing = false;
        wristStage = 0;
    }

    public Pair<Double, Double> getArmPosition() {
        Translation2d vectorUpperArm = new Translation2d(ArmConstants.UPPER_ARM_LENGTH, Rotation2d.fromRadians(winch.getAbsoluteEncoderRad() + Math.PI/2));
        Translation2d vectorForearm = new Translation2d(ArmConstants.FOREARM_LENGTH, Rotation2d.fromRadians(winch.getAbsoluteEncoderRad() + elbow.getAbsoluteEncoderRad() + Math.PI/2));
        Translation2d point = vectorUpperArm.plus(vectorForearm);
        double x = -point.getX();
        double y = point.getY();
        SmartDashboard.putNumber("xAxis", x);
        SmartDashboard.putNumber("yAxis", y);
        return new Pair<>(x, y);
    }

    private double lawOfCosTheta(double a, double b, double c) {
        double result = (Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c, 2)) / (2 * a * b);
        return Math.abs(result) > 1 ? -1 : Math.acos(result);
    }

    public void setSetpoint(double xAxis, double yAxis) {
        double distanceSquared = Math.pow(xAxis, 2) + Math.pow(yAxis, 2);
        double distance = Math.sqrt(distanceSquared);

        if (distance >= ArmConstants.FOREARM_LENGTH + ArmConstants.UPPER_ARM_LENGTH) return;
        else if (distance <= ArmConstants.UPPER_ARM_LENGTH - ArmConstants.FOREARM_LENGTH) return;
        else if (xAxis > ArmConstants.MAX_X_AXIS || xAxis < ArmConstants.MIN_X_AXIS) return;
        else if (yAxis > ArmConstants.MAX_Y_AXIS || yAxis < ArmConstants.MIN_Y_AXIS) return;

        SmartDashboard.putNumber("setX", xAxis);
        SmartDashboard.putNumber("setY", yAxis);
        SmartDashboard.putNumber("lastX", lastX);
        SmartDashboard.putNumber("lastY", lastY);

        double thetaElbow, thetaWinch;
        double l1 = ArmConstants.UPPER_ARM_LENGTH;
        double l2 = ArmConstants.FOREARM_LENGTH;
        double theta1 = lawOfCosTheta(l1, l2, distance);
        thetaElbow = Math.PI - theta1;
        double theta2 = lawOfCosTheta(l1, distance, l2);
        if (Math.abs(yAxis / distance) > 1) return;
        thetaWinch = Math.acos(yAxis / distance) * (xAxis>0? 1: -1) - theta2;

        // Some insurance for the massive arm
        if (theta1 == -1 || theta2 == -1) return;
        else if (thetaElbow <  ArmConstants.MIN_ELBOW_ANGLE || thetaElbow > ArmConstants.MAX_ELBOW_ANGLE) return;
        else if (thetaWinch <  ArmConstants.MIN_WINCH_ANGLE || thetaWinch > ArmConstants.MAX_WINCH_ANGLE) return;
        lastX = xAxis;
        lastY = yAxis;
        isResetting = false;
        desiredElbowAngle = thetaElbow;
        desiredWinchAngle = thetaWinch;
        elbow.setSetpoint(thetaElbow);
        winch.setSetpoint(thetaWinch);
    }

    public boolean atSetpoint() {
        SmartDashboard.putBoolean("atElbowSetpoint", elbow.atSetpoint());
        SmartDashboard.putBoolean("atWinchSetpoint", winch.atSetpoint());
        return elbow.atSetpoint() && winch.atSetpoint();
    }

    public boolean atWinchSetpoint() {
        return winch.atSetpoint();
    }

    private double xAxisMemory = Integer.MAX_VALUE;
    private double yAxisMemory = Integer.MAX_VALUE;

    /**
     * @param xAxis xAxis that the arm will finally move to
     * @param yAxis yAxis that the arm will finally move to
     * You should use this function TWICE.
     * The second one should be called when the first one is got the setpoint
     */
    public void moveTwice(double xAxis, double yAxis) {
        isResetting = false;
        isElbowLocked = true;
        double lastDesiredElbowAngle = desiredElbowAngle;
        double lastDesiredWinchAngle = desiredWinchAngle;
        setSetpoint(xAxis, yAxis);
        if (xAxis == xAxisMemory && yAxis == yAxisMemory) {
            winch.setSetpoint(desiredWinchAngle);
            xAxisMemory = Integer.MAX_VALUE;
            yAxisMemory = Integer.MAX_VALUE;
        } else {
            if (yAxis > getArmPosition().getSecond()){
                winch.setSetpoint(ArmConstants.MIN_WINCH_ANGLE);
                if (desiredElbowAngle < Units.degreesToRadians(90)) {
                    desiredElbowAngle = Units.degreesToRadians(90);
                    desiredWinchAngle = Units.degreesToRadians(0);
                }
            } else {
                winch.setSetpoint(desiredWinchAngle);
                if (desiredElbowAngle > lastDesiredElbowAngle) {
                    desiredElbowAngle = lastDesiredElbowAngle;
                    desiredWinchAngle = lastDesiredWinchAngle;
                }
            }        
            elbow.setSetpoint(desiredElbowAngle - (winch.getAbsoluteEncoderRad() - desiredWinchAngle));        
            if (elbow.getSetpoint() > Units.degreesToRadians(130)) {
                desiredElbowAngle = 130;
                desiredWinchAngle = 0;
            }
            xAxisMemory = xAxis;
            yAxisMemory = yAxis;
        }
    }


    public void setSpeed(double spdX, double spdY) {
        if (isResetting) return;
        isElbowLocked = false;
//          q   Translation2d vectorUpperArm = new Translation2d(ArmConstants.UPPER_ARM_LENGTH, Rotation2d.fromRadians(winch.getAbsoluteEncoderRad() + Math.PI/2));
//        Translation2d vectorForearm = new Translation2d(ArmConstants.FOREARM_LENGTH, Rotation2d.fromRadians(winch.getAbsoluteEncoderRad() + elbow.getAbsoluteEncoderRad() + Math.PI/2));
//        Translation2d point = vectorUpperArm.plus(vectorForearm);
        var armPosition = getArmPosition();
        spdX = MathUtil.clamp(spdX, -1, 1);
        spdY = MathUtil.clamp(spdY, -1, 1);
        if (spdX == 0 && spdY == 0) return;
        if (spdX != 0) lastX = armPosition.getFirst();
        if (spdY != 0) lastY = armPosition.getSecond();
        setSetpoint(lastX+spdX* ArmConstants.xSpdConvertFactor, lastY+spdY* ArmConstants.ySpdConvertFactor);
    }

    public void addWristStage(int stage) {
        wristStage += stage;
    }

    public boolean getIsResetting()  {
        return isResetting;
    }
//
//    public void setArmSpeed(double speed) {
//        if(speed == 0) return;
//        elbow.setSetpoint(elbow.getEncoder() + speed/ElevatorConstants.P_ELBOW);
//    }
//
    public boolean getIsTransporting() {
        return isTransporting;
    }

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


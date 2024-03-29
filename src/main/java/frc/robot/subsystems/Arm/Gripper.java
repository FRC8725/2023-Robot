package frc.robot.subsystems.Arm;


import javax.sound.sampled.Line;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.LazySparkMax;
import frc.lib.LazyTalonFX;
import frc.robot.RobotMap.ArmPort;
import frc.robot.Constants.ArmConstants;

import java.sql.Driver;

public class Gripper {

    private final static Gripper INSTANCE = new Gripper();

    @SuppressWarnings("WeakerAccess")
    public static Gripper getInstance() {
        return INSTANCE;
    }

//    LazySparkMax intakeLeader, intakeFollower;
//    LazyTalonFX rollMotor;

    ProfiledPIDController rollProfiledPIDController;
    private final Rev2mDistanceSensor distanceSensor;
    double lastRange;
    boolean isEnable;
    double range;
    int status;

    private Gripper() {
//        rollMotor = new LazyTalonFX(ArmPort.ROLL_MOTOR, ArmConstants.ROLL_MOTOR_GEAR_RATIO);
//        rollMotor = new LazySparkMax(ElevatorPort.ROLL_MOTOR, ElevatorConstants.ROLL_MOTOR_GEAR_RATIO);
//        rollMotor.setNeutralMode(NeutralMode.Brake);
//        rollMotor.setSelectedSensorPosition(0);
//
//        intakeLeader = new LazySparkMax(ArmPort.INTAKE_LEADER_MOTOR, ArmConstants.INTAKE_GEAR_RATIO);
//        intakeLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
//        intakeFollower = new LazySparkMax(ArmPort.INTAKE_FOLLOWER_MOTOR, ArmConstants.INTAKE_GEAR_RATIO);
//        intakeFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
//        intakeFollower.follow(intakeLeader, true);
//
//        rollProfiledPIDController = new ProfiledPIDController(ArmConstants.P_ROLL, ArmConstants.I_ROLL, ArmConstants.D_ROLL, ArmConstants.ROLL_CONTROLLER_CONSTRAINTS);
//        rollProfiledPIDController.setTolerance(ArmConstants.PID_ROLL_ANGULAR_TOLERANCE_RADS);
//        rollProfiledPIDController.disableContinuousInput();

        distanceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kMXP);
//        distanceSensor.setEnabled(true);
//        distanceSensor.setAutomaticMode(true);
//        distanceSensor.setDistanceUnits(Unit.kInches);
    //    distanceSensor.setRangeProfile(RangeProfile.kDefault);
    //    distanceSensor.setMeasurementPeriod(0.2);

        lastRange = 0;
        range = 0;
        status = 0;
        isEnable = true;
    }

    

    public void autoSwitchDistance() {
        if (DriverStation.isDisabled() && status == 0) {
            status = 1;
            distanceSensor.setAutomaticMode(false);
        }
        if (DriverStation.isEnabled() && status == 1) {
            status = 2;
            distanceSensor.setAutomaticMode(true);
        }
        if (distanceSensor.isRangeValid() && DriverStation.isEnabled()) range = distanceSensor.getRange();
    }

//     public void setDistanceEnable(boolean enable) {
//         if (distanceSensor.isEnabled() != enable) distanceSensor.setEnabled(enable);
//     }

    public void putSmartDashboard() {
//        SmartDashboard.putNumber("distance", distanceSensor.getRange());
        SmartDashboard.putBoolean("isDistanceEnable", isEnable);
    }


    public double getDistanceSensor() {
//        return distanceSensor.getRange();
        // if (!distanceSensor.isEnabled()) distanceSensor.setEnabled(true);
        if (!isEnable || range == lastRange) return 78.74;
        lastRange = range;
        return range;
    }

    public boolean isDistanceRangeValid() {
        return distanceSensor.isRangeValid();
    }

    public void disableDistance() {
        distanceSensor.setEnabled(false);
        isEnable = false;
    }

//    public void calculate() {
////        rollMotor.set(MathUtil.clamp(rollProfiledPIDController.calculate(rollMotor.getPositionAsRad() + Units.degreesToRadians(gyro.getRoll())), -ElevatorConstants.MAX_ROLL_SPEED, ElevatorConstants.MAX_ROLL_SPEED));
//        rollMotor.set(MathUtil.clamp(rollProfiledPIDController.calculate(rollMotor.getPositionAsRad()), -ArmConstants.MAX_ROLL_SPEED, ArmConstants.MAX_ROLL_SPEED));
//    }
//
//    public double getRollSetpoint() {
//        return rollProfiledPIDController.getGoal().position;
//    }
//
//    public boolean atRollSetpoint() {
//        return Math.abs(setpoint - rollMotor.getPositionAsRad()) < ArmConstants.PID_ROLL_ANGULAR_TOLERANCE_RADS;
//    }
//
//    public void setRollSetpoint(double setpoint) {
//        rollProfiledPIDController.setGoal(setpoint);
//        this.setpoint = setpoint;
//    }
//
//    public double getRollEncoder() {
//        return rollMotor.getPositionAsRad();
//    }
//
//    public void run(boolean isRun, boolean isInverted) {
//        intakeLeader.set((isRun? ArmConstants.INTAKE_SPEED : 0) * (isInverted? -1: 1));
//    }
}


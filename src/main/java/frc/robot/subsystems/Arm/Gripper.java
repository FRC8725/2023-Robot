package frc.robot.subsystems.Arm;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.LazySparkMax;
import frc.lib.LazyTalonFX;
import frc.robot.RobotMap.ArmPort;
import frc.robot.Constants.ArmConstants;

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

    private Gripper() {
        Timer.delay(2);
//        rollMotor = new LazyTalonFX(ArmPort.ROLL_MOTOR, ArmConstants.ROLL_MOTOR_GEAR_RATIO);
////        rollMotor = new LazySparkMax(ElevatorPort.ROLL_MOTOR, ElevatorConstants.ROLL_MOTOR_GEAR_RATIO);
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
        distanceSensor.setAutomaticMode(true);
        distanceSensor.setRangeProfile(com.revrobotics.Rev2mDistanceSensor.RangeProfile.kHighSpeed);
    }

    public void setDistanceEnable(boolean enable) {
        if (distanceSensor.isEnabled() != enable) distanceSensor.setEnabled(enable);
    }


    public double getDistanceSensor() {
//        return distanceSensor.getRange();
        if (!distanceSensor.isEnabled()) distanceSensor.setEnabled(true);
        double range = 2000;
        if (distanceSensor.isRangeValid()) range = distanceSensor.getRange(Unit.kMillimeters);
        return range;
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


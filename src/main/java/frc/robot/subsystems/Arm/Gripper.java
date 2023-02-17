package frc.robot.subsystems.Arm;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.lib.LazySparkMax;
import frc.robot.RobotMap.ArmPort;
import frc.robot.Constants.ArmConstants;

public class Gripper {

    private final static Gripper INSTANCE = new Gripper();

    @SuppressWarnings("WeakerAccess")
    public static Gripper getInstance() {
        return INSTANCE;
    }

    LazySparkMax intakeLeader, intakeFollower;
    LazySparkMax rollMotor;

    ProfiledPIDController rollProfiledPIDController;

    private Gripper() {
        rollMotor = new LazySparkMax(ArmPort.kRollMotor, ArmConstants.kRollMotorGearRatio);
//        rollMotor = new LazySparkMax(ElevatorPort.kRollMotor, ElevatorConstants.kRollMotorGearRatio);
        rollMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        intakeLeader = new LazySparkMax(ArmPort.kIntakeLeaderMotor, ArmConstants.kIntakeGearRatio);
        intakeLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
        intakeFollower = new LazySparkMax(ArmPort.kIntakeFollowerMotor, ArmConstants.kIntakeGearRatio);
        intakeFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
        intakeFollower.follow(intakeLeader, true);

        rollProfiledPIDController = new ProfiledPIDController(ArmConstants.kPRoll, ArmConstants.kIRoll, ArmConstants.kDRoll, ArmConstants.kRollControllerConstraints);
        rollProfiledPIDController.setTolerance(ArmConstants.kPIDRollAngularToleranceRads);
        rollProfiledPIDController.disableContinuousInput();
    }

    public void calculate() {
        if (atRollSetpoint()) rollMotor.set(0);
//        rollMotor.set(MathUtil.clamp(rollProfiledPIDController.calculate(rollMotor.getPositionAsRad() + Units.degreesToRadians(gyro.getRoll())), -ElevatorConstants.kMaxRollSpeed, ElevatorConstants.kMaxRollSpeed));
        rollMotor.set(MathUtil.clamp(rollProfiledPIDController.calculate(rollMotor.getPositionAsRad()), -ArmConstants.kMaxRollSpeed, ArmConstants.kMaxRollSpeed));
    }

    public double getRollSetpoint() {
        return rollProfiledPIDController.getGoal().position;
    }

    public boolean atRollSetpoint() {
        return Math.abs(rollProfiledPIDController.getSetpoint().position - rollMotor.getPositionAsRad()) < ArmConstants.kPIDRollAngularToleranceRads;
    }

    public void setRollSetpoint(double setpoint) {
        rollProfiledPIDController.setGoal(setpoint);
    }

    public double getRollEncoder() {
        return rollMotor.getPositionAsRad();
    }

    public void run(boolean isRun, boolean isInverted) {
        intakeLeader.set((isRun? ArmConstants.kIntakeSpeed: 0) * (isInverted? -1: 1));
    }
}


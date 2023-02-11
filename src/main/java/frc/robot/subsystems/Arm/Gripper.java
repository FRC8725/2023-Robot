package frc.robot.subsystems.Arm;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LazySparkMax;
import frc.lib.LazyTalonFX;
import frc.robot.RobotMap.ElevatorPort;
import frc.robot.Constants.ElevatorConstants;

public class Gripper extends SubsystemBase {

    private final static Gripper INSTANCE = new Gripper();

    @SuppressWarnings("WeakerAccess")
    public static Gripper getInstance() {
        return INSTANCE;
    }

    LazySparkMax intakeLeader, intakeFollower;
    LazyTalonFX rollMotor;

    ProfiledPIDController rollProfiledPIDController;

    DutyCycleEncoder absoluteEncoder;

    AHRS gyro = new AHRS();

    private Gripper() {
        rollMotor = new LazyTalonFX(ElevatorPort.kRollMotor, ElevatorConstants.kRollMotorGearRatio);
//        rollMotor = new LazySparkMax(ElevatorPort.kRollMotor, ElevatorConstants.kRollMotorGearRatio);
        rollMotor.setNeutralMode(NeutralMode.Coast);

        intakeLeader = new LazySparkMax(ElevatorPort.kIntakeLeaderMotor, ElevatorConstants.kIntakeGearRatio);
        intakeLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
        intakeFollower = new LazySparkMax(ElevatorPort.kIntakeFollowerMotor, ElevatorConstants.kIntakeGearRatio);
        intakeFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
        intakeFollower.follow(intakeLeader, true);

        absoluteEncoder = new DutyCycleEncoder(ElevatorPort.kWristAbsoluteEncoder);
        absoluteEncoder.setPositionOffset(ElevatorConstants.kWristAbsoluteEncoderOffset);

        rollProfiledPIDController = new ProfiledPIDController(ElevatorConstants.kPRoll, ElevatorConstants.kIRoll, ElevatorConstants.kDRoll, ElevatorConstants.kRollControllerConstraints);
        rollProfiledPIDController.setTolerance(ElevatorConstants.kPIDRollAngularToleranceRads);
        rollProfiledPIDController.disableContinuousInput();
        rollMotor.setRadPosition(0);
    }

    @Override
    public void periodic() {
        rollMotor.set(rollProfiledPIDController.calculate(rollMotor.getPositionAsRad() + Units.degreesToRadians(gyro.getRoll())));
    }

    public double getAbsoluteEncoderRad() {
        double measurement = absoluteEncoder.getAbsolutePosition()-absoluteEncoder.getPositionOffset();
        if (Math.abs(measurement) > 0.5) measurement += measurement < 0? 1: -1;
        return measurement*2*Math.PI*(ElevatorConstants.kWristAbosoluteEncoderInverted? -1: 1);
    }

    public double getRollSetpoint() {
        return rollProfiledPIDController.getGoal().position;
    }

    public void setRollSetpoint(double setpoint) {
        rollProfiledPIDController.setGoal(setpoint);
    }

    public double getRollEncoder() {
        return rollMotor.getPositionAsRad();
    }

    public void run(boolean isRun, boolean isInverted) {
        intakeLeader.set((isRun? ElevatorConstants.kIntakeSpeed: 0) * (isInverted? -1: 1));
    }
}


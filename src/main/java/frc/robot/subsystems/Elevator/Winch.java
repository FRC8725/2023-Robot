package frc.robot.subsystems.Elevator;


import com.fasterxml.jackson.core.sym.NameN;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LazySparkMax;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotMap.ElevatorPort;

public class Winch extends SubsystemBase {

    private final static Winch INSTANCE = new Winch();

    @SuppressWarnings("WeakerAccess")
    public static Winch getInstance() {
        return INSTANCE;
    }

    LazySparkMax winchMotor;

    ProfiledPIDController winchProfiledPIDController;
    DutyCycleEncoder absoluteEncoder;

    private Winch() {
        winchMotor = new LazySparkMax(ElevatorPort.kWinchMotor, ElevatorConstants.kWinchGearRatio);
        winchMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        winchProfiledPIDController = new ProfiledPIDController(ElevatorConstants.kPWinch, ElevatorConstants.kIWinch, ElevatorConstants.kDWinch, ElevatorConstants.kWinchControllerConstraints);
        winchProfiledPIDController.setTolerance(ElevatorConstants.kPIDWinchAngularToleranceRads);
        winchProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);

        absoluteEncoder = new DutyCycleEncoder(ElevatorPort.kWinchAbsoluteEncoder);
        absoluteEncoder.setPositionOffset(ElevatorConstants.kWinchAbsoluteEncoderOffset);
        resetEncoder();
    }

    @Override
    public void periodic() {
        winchMotor.set(MathUtil.clamp(winchProfiledPIDController.calculate(winchMotor.getPositionAsRad()), -ElevatorConstants.kMaxWinchSpeed, ElevatorConstants.kMaxWinchSpeed));
    }

    public void resetEncoder() {
        winchMotor.setRadPosition(getAbsoluteEncoderRad());
    }

    public double getAbsoluteEncoderRad() {
        double measurement = absoluteEncoder.getAbsolutePosition();
        return measurement*2*Math.PI;
    }

    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ElevatorConstants.kMinWinchAngle, ElevatorConstants.kMaxWinchAngle);
        SmartDashboard.putNumber("Winch Setpoint", setpoint);
        winchProfiledPIDController.setGoal(setpoint);
    }

    public boolean atSetpoint() {
        return winchProfiledPIDController.atSetpoint();
    }

    public double getEncoder() {
        return winchMotor.getPositionAsRad();
    }

    public void stop() {
        winchMotor.set(0);
        winchProfiledPIDController.setGoal(winchMotor.getPositionAsRad());
    }
}


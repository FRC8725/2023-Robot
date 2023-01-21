package frc.robot.subsystems.Elevator;


import com.ctre.phoenix.sensors.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
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

        absoluteEncoder = new DutyCycleEncoder(ElevatorPort.kWinchAbsoluteEncoder);
        absoluteEncoder.setDistancePerRotation(360);
    }

    @Override
    public void periodic() {
        winchMotor.set(winchProfiledPIDController.calculate(getAbsoluteEncoderRad()));
    }

    public double getAbsoluteEncoderRad() {
        double measurement = absoluteEncoder.getAbsolutePosition()-ElevatorConstants.kWinchAbsoluteEncoderOffsetAngle;
        return measurement/180*Math.PI;
    }

    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, ElevatorConstants.kMinWinchAngle, ElevatorConstants.kMaxWinchAngle);
        winchProfiledPIDController.setGoal(setpoint);
    }

    public double getSetpoint() {
        return winchProfiledPIDController.getSetpoint().position;
    }

    public void stop() {
        winchMotor.set(0);
        winchProfiledPIDController.setGoal(getAbsoluteEncoderRad());
    }
}


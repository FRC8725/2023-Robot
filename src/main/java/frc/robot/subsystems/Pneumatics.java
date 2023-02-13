package frc.robot.subsystems;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Pneumatics extends SubsystemBase {

    private final static Pneumatics INSTANCE = new Pneumatics();
    Compressor compressor;

    DoubleSolenoid gripperPressureSwitcher;
    DoubleSolenoid gripperReleaser;

    PneumaticHub pneumaticHub;

    private Pneumatics() {
        PneumaticsModuleType moduleType = PneumaticsModuleType.REVPH;
        pneumaticHub = new PneumaticHub(RobotMap.PneumaticsPort.kREVPHPort);
        compressor = new Compressor(RobotMap.PneumaticsPort.kREVPHPort, moduleType);
        gripperPressureSwitcher = new DoubleSolenoid(RobotMap.PneumaticsPort.kREVPHPort, moduleType,
        RobotMap.ElevatorPort.kGripperPressureSwitcherDoubleSolenoid[0],
        RobotMap.ElevatorPort.kGripperPressureSwitcherDoubleSolenoid[1]);
        gripperPressureSwitcher.set(DoubleSolenoid.Value.kForward);
        gripperReleaser = new DoubleSolenoid(RobotMap.PneumaticsPort.kREVPHPort, moduleType,
        RobotMap.ElevatorPort.kGripperReleaseDoubleSolenoid[0], 
        RobotMap.ElevatorPort.kGripperReleaseDoubleSolenoid[1]);
        gripperReleaser.set(DoubleSolenoid.Value.kReverse);
    }

    @SuppressWarnings("WeakerAccess")
    public static Pneumatics getInstance() {
        return INSTANCE;
    }

    @Override
    public void periodic() {
        compressor.enableDigital();
    }

    public void setGripper(boolean isOpen, boolean isHighPressure) {
        SmartDashboard.putBoolean("isGripperOpen", isOpen);
        gripperPressureSwitcher.set(isHighPressure? DoubleSolenoid.Value.kForward: DoubleSolenoid.Value.kReverse);
        gripperReleaser.set(isOpen? DoubleSolenoid.Value.kForward: DoubleSolenoid.Value.kReverse);
    }

    /**
     * @return isOpen
     */
    public boolean getGripperStatus() {
        return gripperReleaser.get() != DoubleSolenoid.Value.kForward;
    }

    public boolean isHighPressure() {
        return gripperPressureSwitcher.get() == DoubleSolenoid.Value.kForward;
    }

}


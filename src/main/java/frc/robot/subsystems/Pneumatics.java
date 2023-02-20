package frc.robot.subsystems;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Pneumatics extends SubsystemBase {

    private final static Pneumatics INSTANCE = new Pneumatics();
    Compressor compressor;

//    DoubleSolenoid gripperPressureSwitcher;
    DoubleSolenoid gripperReleaser;

    PneumaticHub pneumaticHub;

    private Pneumatics() {
        PneumaticsModuleType moduleType = PneumaticsModuleType.REVPH;
        pneumaticHub = new PneumaticHub(RobotMap.PneumaticsPort.REV_PH_PORT);
        compressor = new Compressor(RobotMap.PneumaticsPort.REV_PH_PORT, moduleType);
//        gripperPressureSwitcher = new DoubleSolenoid(RobotMap.PneumaticsPort.REV_PH_PORT, moduleType,
//        RobotMap.ArmPort.GRIPPER_PRESSURE_SWITCHER_DOUBLE_SOLENOID[0],
//        RobotMap.ArmPort.GRIPPER_PRESSURE_SWITCHER_DOUBLE_SOLENOID[1]);
//        gripperPressureSwitcher.set(DoubleSolenoid.Value.kForward);
        gripperReleaser = new DoubleSolenoid(RobotMap.PneumaticsPort.REV_PH_PORT, moduleType,
        RobotMap.ArmPort.GRIPPER_RELEASE_DOUBLE_SOLENOID[0],
        RobotMap.ArmPort.GRIPPER_RELEASE_DOUBLE_SOLENOID[1]);
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

    public void setGripper(boolean isOpen) {
        SmartDashboard.putBoolean("isGripperOpen", isOpen);
//        gripperPressureSwitcher.set(isHighPressure? DoubleSolenoid.Value.kForward: DoubleSolenoid.Value.kReverse);
        gripperReleaser.set(isOpen? DoubleSolenoid.Value.kForward: DoubleSolenoid.Value.kReverse);
    }

    /**
     * @return isOpen
     */
    public boolean getGripperStatus() {
        return gripperReleaser.get() != DoubleSolenoid.Value.kForward;
    }

//    public boolean isHighPressure() {
//        return gripperPressureSwitcher.get() == DoubleSolenoid.Value.kForward;
//    }

}


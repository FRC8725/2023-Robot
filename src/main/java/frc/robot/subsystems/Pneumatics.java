package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Pneumatics extends SubsystemBase {

    private final static Pneumatics INSTANCE = new Pneumatics();
    Compressor compressor;

    DoubleSolenoid winchReleaser;
    DoubleSolenoid gripperIntrance;

    PneumaticHub pneumaticHub;

    private Pneumatics() {
        PneumaticsModuleType moduleType = PneumaticsModuleType.REVPH;
        pneumaticHub = new PneumaticHub(RobotMap.PneumaticsPort.REV_PH_PORT);
        compressor = new Compressor(RobotMap.PneumaticsPort.REV_PH_PORT, moduleType);
        winchReleaser = new DoubleSolenoid(RobotMap.PneumaticsPort.REV_PH_PORT, moduleType,
        RobotMap.ElevatorPort.WINCH_RELEASE_DOUBLE_SOLENOID[0],
        RobotMap.ElevatorPort.WINCH_RELEASE_DOUBLE_SOLENOID[1]);
        winchReleaser.set(DoubleSolenoid.Value.kReverse);
        gripperIntrance = new DoubleSolenoid(RobotMap.PneumaticsPort.REV_PH_PORT, moduleType,
        RobotMap.ElevatorPort.GRIPPER_RELEASE_DOUBLE_SOLENOID[0],
        RobotMap.ElevatorPort.GRIPPER_RELEASE_DOUBLE_SOLENOID[1]);
        gripperIntrance.set(DoubleSolenoid.Value.kReverse);
    }

    @SuppressWarnings("WeakerAccess")
    public static Pneumatics getInstance() {
        return INSTANCE;
    }

    @Override
    public void periodic() {
        compressor.enableDigital();
    }

    public void toggleArm() {
        winchReleaser.toggle();
    }

    public void setGripper(boolean isOpen) {
        SmartDashboard.putBoolean("isGripperOpen", isOpen);
        gripperIntrance.set(isOpen? DoubleSolenoid.Value.kForward: DoubleSolenoid.Value.kReverse);
    }

    /**
     * @return isOpen
     */
    public boolean getGripperStatus() {
        return gripperIntrance.get() != DoubleSolenoid.Value.kForward;
    }

}


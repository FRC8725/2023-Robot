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
    DoubleSolenoid gripperEntrance;

    PneumaticHub pneumaticHub;

    private Pneumatics() {
        PneumaticsModuleType moduleType = PneumaticsModuleType.REVPH;
        pneumaticHub = new PneumaticHub(RobotMap.PneumaticsPort.kREVPHPort);
        compressor = new Compressor(RobotMap.PneumaticsPort.kREVPHPort, moduleType);
        winchReleaser = new DoubleSolenoid(RobotMap.PneumaticsPort.kREVPHPort, moduleType, 
        RobotMap.ElevatorPort.kWinchReleaseDoubleSolenoid[0], 
        RobotMap.ElevatorPort.kWinchReleaseDoubleSolenoid[1]);
        winchReleaser.set(DoubleSolenoid.Value.kReverse);
        gripperEntrance = new DoubleSolenoid(RobotMap.PneumaticsPort.kREVPHPort, moduleType,
        RobotMap.ElevatorPort.kGripperReleaseDoubleSolenoid[0], 
        RobotMap.ElevatorPort.kGripperReleaseDoubleSolenoid[1]);
        gripperEntrance.set(DoubleSolenoid.Value.kReverse);
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
        gripperEntrance.set(isOpen? DoubleSolenoid.Value.kForward: DoubleSolenoid.Value.kReverse);
    }

    /**
     * @return isOpen
     */
    public boolean getGripperStatus() {
        return gripperEntrance.get() != DoubleSolenoid.Value.kForward;
    }

}


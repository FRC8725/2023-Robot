package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Pneumatics extends SubsystemBase {

    private final static Pneumatics INSTANCE = new Pneumatics();
    Compressor compressor;

    DoubleSolenoid winchReleaser;
    DoubleSolenoid gripperIntrance;


    private Pneumatics() {
        PneumaticsModuleType moduleType = PneumaticsModuleType.REVPH;
        compressor = new Compressor(0, moduleType);
        int forwardChannel = RobotMap.ElevatorPort.kWinchReleaseDoubleSolenoid[0];
        int reverseChannel = RobotMap.ElevatorPort.kWinchReleaseDoubleSolenoid[1];
        winchReleaser = new DoubleSolenoid(moduleType, forwardChannel, reverseChannel);
        winchReleaser.set(DoubleSolenoid.Value.kReverse);
        forwardChannel = RobotMap.ElevatorPort.kGripperReleaseDoubleSolenoid[0];
        reverseChannel = RobotMap.ElevatorPort.kGripperReleaseDoubleSolenoid[1];
        gripperIntrance = new DoubleSolenoid(moduleType, forwardChannel, reverseChannel);
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

    public void toggleGripperOpen() {
        gripperIntrance.toggle();
    }

}


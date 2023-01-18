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


    private Pneumatics() {
        compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        int forwardChannel= RobotMap.ElevatorPort.kWinchReleaseDoubleSolenoid[0];
        int reverseChannel = RobotMap.ElevatorPort.kWinchReleaseDoubleSolenoid[1];
        winchReleaser = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, forwardChannel, reverseChannel);
        winchReleaser.set(DoubleSolenoid.Value.kReverse);
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

}


package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Paneumatics extends SubsystemBase {

    private final static Paneumatics INSTANCE = new Paneumatics();

    @SuppressWarnings("WeakerAccess")
    public static Paneumatics getInstance() {
        return INSTANCE;
    }

    Compressor compressor;
    private Paneumatics() {
        compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    }

    @Override
    public void periodic() {
        compressor.enableDigital();
    }


}


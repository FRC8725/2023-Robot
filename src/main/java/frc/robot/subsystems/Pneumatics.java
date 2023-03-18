package frc.robot.subsystems;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Pneumatics extends SubsystemBase {

    private final static Pneumatics INSTANCE = new Pneumatics();
    Compressor compressor;

    PneumaticHub pneumaticHub;

//    DoubleSolenoid gripperPressureSwitcher;
    DoubleSolenoid gripperReleaser;

    boolean isFirstLoop;
    Timer gameTimer = new Timer();
    Timer pneumaticTimer = new Timer();


    private Pneumatics() {
        PneumaticsModuleType moduleType = PneumaticsModuleType.REVPH;
        compressor = new Compressor(RobotMap.PneumaticsPort.PNEUMATIC_HUB_PORT, moduleType);
        pneumaticHub = new PneumaticHub(RobotMap.PneumaticsPort.PNEUMATIC_HUB_PORT);
//        gripperPressureSwitcher = new DoubleSolenoid(RobotMap.PneumaticsPort.REV_PH_PORT, moduleType,
//        RobotMap.ArmPort.GRIPPER_PRESSURE_SWITCHER_DOUBLE_SOLENOID[0],
//        RobotMap.ArmPort.GRIPPER_PRESSURE_SWITCHER_DOUBLE_SOLENOID[1]);
//        gripperPressureSwitcher.set(DoubleSolenoid.Value.kForward);
        gripperReleaser = new DoubleSolenoid(RobotMap.PneumaticsPort.PNEUMATIC_HUB_PORT, moduleType,
        RobotMap.ArmPort.GRIPPER_RELEASE_DOUBLE_SOLENOID[0],
        RobotMap.ArmPort.GRIPPER_RELEASE_DOUBLE_SOLENOID[1]);
        gripperReleaser.set(DoubleSolenoid.Value.kReverse);
        gameTimer.stop();
        gameTimer.reset();
        pneumaticTimer.stop();
        pneumaticTimer.reset();
        isFirstLoop = true;
    }

    @SuppressWarnings("WeakerAccess")
    public static Pneumatics getInstance() {
        return INSTANCE;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isGripperOpen", getGripperStatus());
        SmartDashboard.putNumber("pneumaticTimer", pneumaticTimer.get());

        if (isFirstLoop) {
            if (!pneumaticHub.getPressureSwitch()) {
                isFirstLoop = false;
                gameTimer.start();
            }
        } else {
            if (compressor.isEnabled()) pneumaticTimer.start();
            else pneumaticTimer.stop();
        }

        if (pneumaticTimer.get() > 36) compressor.disable();
        else compressor.enableDigital();

        if (gameTimer.get() > 180) {
            isFirstLoop = true;
            gameTimer.stop();
            gameTimer.reset();
            pneumaticTimer.stop();
            pneumaticTimer.reset();
        }
    }

    public void setGripper(boolean isOpen) {
//        gripperPressureSwitcher.set(isHighPressure? DoubleSolenoid.Value.kForward: DoubleSolenoid.Value.kReverse);
        gripperReleaser.set(isOpen? DoubleSolenoid.Value.kForward: DoubleSolenoid.Value.kReverse);
    }

    public void toggleGripper() {
        gripperReleaser.toggle();
    }

    /**
     * @return isOpen
     */
    public boolean getGripperStatus() {
        return gripperReleaser.get() == DoubleSolenoid.Value.kForward;
    }

//    public boolean isHighPressure() {
//        return gripperPressureSwitcher.get() == DoubleSolenoid.Value.kForward;
//    }

}


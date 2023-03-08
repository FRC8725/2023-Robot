package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.RobotMap;

public final class LEDSubsystem extends SubsystemBase {

    private static final LEDSubsystem INSTANCE = new LEDSubsystem();

    public static LEDSubsystem getInstance() {
        return INSTANCE;
    }
    AddressableLED frontLEDs, backLEDs;
    AddressableLEDBuffer frontLEDBuffer, backLEDBuffer;

    private LEDSubsystem() {
        frontLEDs = new AddressableLED(RobotMap.LEDPort.LED_FRONT_PORT);
        backLEDs = new AddressableLED(RobotMap.LEDPort.LED_BACK_PORT);
        frontLEDBuffer = new AddressableLEDBuffer(LEDConstants.BUFFER_SIZE);
        backLEDBuffer = new AddressableLEDBuffer(LEDConstants.BUFFER_SIZE);
        frontLEDs.setLength(LEDConstants.BUFFER_SIZE);
        frontLEDs.start();
        backLEDs.setLength(LEDConstants.BUFFER_SIZE);
        backLEDs.start();
    }

    @Override
    public void periodic() {
        frontLEDs.setData(frontLEDBuffer);
        backLEDs.setData(backLEDBuffer);
    }

    public void setFrontColor(Color color, int part) {
        for (var i = frontLEDBuffer.getLength() / 2 * part;
             i < frontLEDBuffer.getLength() - (frontLEDBuffer.getLength() / 2) * part;
             i++) {
            frontLEDBuffer.setLED(i, color);
        }
    }

    public void setBackColor(Color color) {
        for (var i = 0; i < backLEDBuffer.getLength(); i++) {
            backLEDBuffer.setLED(i, color);
        }
    }

    private int rainbowFirstPixelHue = 0;

    // From Example LED Project
    public void rainbow() {
        // For every pixel
        for (var i = 0; i < LEDConstants.BUFFER_SIZE; i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            var hue = (rainbowFirstPixelHue + (i * 180 / (LEDConstants.BUFFER_SIZE * 2))) % 180;
            // Set the value
            frontLEDBuffer.setHSV(i, hue, 255, 128);
        }

        // For every pixel
        for (var i = 0; i < LEDConstants.BUFFER_SIZE; i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            var hue = (rainbowFirstPixelHue + (i * 180 / (LEDConstants.BUFFER_SIZE * 2))) % 180;
            // Set the value
            backLEDBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

}


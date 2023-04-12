package frc.robot.subsystems;

import org.frc5587.lib.advanced.RainbowLEDPattern;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {
    private final AddressableLED leds;
    public final AddressableLEDBuffer buffer;
    private final RainbowLEDPattern rainbowLEDPattern = new RainbowLEDPattern(90, LEDConstants.STRIP_LENGTH, LEDConstants.STRIP_LENGTH, 20);
    private boolean isRunningRainbow = true;
    private int patternIndexer = 0;
    private double timeSinceSetSeconds = 0;
    private enum LEDPattern {
        Rainbow,
        Solid
    }

    private SendableChooser<LEDPattern> ledChooser = new SendableChooser<LEDPattern>();

    public LEDs() {
        leds = new AddressableLED(LEDConstants.PORT);

        buffer = new AddressableLEDBuffer(LEDConstants.STRIP_LENGTH);
        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
        ledChooser.addOption("Rainbow", LEDPattern.Rainbow);
        ledChooser.setDefaultOption("Solid", LEDPattern.Solid);
        SmartDashboard.putData("LED Pattern:", ledChooser);
    }

    public void setColor(int r, int g, int b) {
        isRunningRainbow = false;
        timeSinceSetSeconds = 0;
        for(int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
    }

    public void setYellow() {
        setColor(255, 255, 0);
    }

    public void setPurple() {
        setColor(178, 37, 188);
    }

    public void off() {
        setColor(0, 0, 0);
    }

    public void setRed() {
        setColor(255, 0, 0);
    }

    public void setBlue() {
        setColor(0, 0, 255);
    }

    public void setWhite() {
        setColor(255, 255, 255);
    }

    public void setRainbow() {
        isRunningRainbow = true;
        patternIndexer = 0;
    }

    @Override
    public void periodic() {
        if(DriverStation.isEnabled()) {
            if(timeSinceSetSeconds >= 10) {
                timeSinceSetSeconds = 0;
                isRunningRainbow = true;
            }
            timeSinceSetSeconds += 0.02;
        }

        switch(ledChooser.getSelected()) {
            case Solid:
                break;
            case Rainbow:
                setRainbow();
                break;
            default:
                setRainbow();
                break;
        }
        if(!isRunningRainbow) {
            leds.setData(buffer);
        }
        else {
            leds.setData(rainbowLEDPattern.step(patternIndexer, buffer));
            patternIndexer++;
        }
    }
}

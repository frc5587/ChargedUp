package frc.robot.subsystems;

import org.frc5587.lib.advanced.RainbowLEDPattern;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.RedBlueChase;

public class LEDs extends SubsystemBase {
    private final AddressableLED leds;
    public final AddressableLEDBuffer buffer;
    private final RainbowLEDPattern rainbowLEDPattern = new RainbowLEDPattern(30, 60, 60, 55);
    private final RedBlueChase chase = new RedBlueChase(60, 60, 55);
    public boolean isRunningRainbow = false;
    public boolean isRunningChase = true;
    private int patternIndexer = 0;
    private double timeSinceSetSeconds = 0;

    public LEDs() {
        leds = new AddressableLED(LEDConstants.PORT);

        buffer = new AddressableLEDBuffer(LEDConstants.STRIP_LENGTH);
        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
    }

    public void setColor(int r, int g, int b) {
        isRunningRainbow = false;
        isRunningChase = false;
        timeSinceSetSeconds = 0;
        for(int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
    }

    public void setYellow() {
        isRunningChase = false;
        isRunningRainbow = false;
        setColor(255, 255, 0);
    }

    public void setPurple() {
        isRunningChase = false;
        isRunningRainbow = false;
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
        isRunningChase = false;
        patternIndexer = 0;
    }

    public void setChase() {
        isRunningChase = true;
        isRunningRainbow = false;
        patternIndexer = 0;
    }

    @Override
    public void periodic() {
        timeSinceSetSeconds += 0.02;
        System.out.println(timeSinceSetSeconds);
        if(timeSinceSetSeconds >= 30) {
            timeSinceSetSeconds = 0;
            isRunningRainbow = true;
        }
        if(!isRunningRainbow && !isRunningChase) {
            leds.setData(buffer);
        }
        else if(isRunningChase) {
            leds.setData(chase.step(patternIndexer, buffer));
            patternIndexer++;
        }
        else if(isRunningRainbow) {
            leds.setData(rainbowLEDPattern.step(patternIndexer, buffer));
            patternIndexer++;
        }
    }
}

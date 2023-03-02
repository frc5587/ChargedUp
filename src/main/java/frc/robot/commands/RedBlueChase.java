package frc.robot.commands;

import org.frc5587.lib.advanced.CustomLEDPattern;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class RedBlueChase implements CustomLEDPattern {
    private static final double stepTime = 0.02;
    private double idx = 0;
    private int speed, brightness, length;
    private int[] redLEDs, blueLEDs;

    /**
     * Creates the rainbow pattern LED controller. You can set `waveLength` and `length` to the same value if you want to "stretch" the rainbow across the entire strip (it looks kinda fire)
     * 
     * @param speed how fast the pattern travels across the strip, in pixels per second
     * @param wavelength length of the wave, so distance (in pixels) from red to red
     * @param length length of LED strip, in pixels
     * @param brightness from 0 to 255, its the brightness of the LED strip
     */
    public RedBlueChase(int speed, int length, int brightness) {
        this.speed = speed;
        this.brightness = brightness;
        this.length = length;
    }

    @Override
    public AddressableLEDBuffer step(int stepNumber, AddressableLEDBuffer ledBuffer) {
        if(idx < 0) {
            idx = length;
        }

        updateLEDLists();
        
        for (int i = 0; i < redLEDs.length; i++) {
            ledBuffer.setRGB(redLEDs[i], brightness, 0, 0);
        }
        for (int i = 0; i < blueLEDs.length; i++) {
            ledBuffer.setRGB(blueLEDs[i], 0, 0, brightness);
        }
        
        idx -= stepTime * speed;

        return ledBuffer;
    }

    public void updateLEDLists() {
        redLEDs = new int[length/2+1];
        blueLEDs = new int[length/2+1];
        int groupLength = length / 2 + (int) idx;
        int redArrSetterIndex = 0;
        int blueArrSetterIndex = 0;
        for (int i = 0; i < length; i++) {
            if((i > idx && i < groupLength) || (i < groupLength - length)) {
                // System.out.println("THIS IS A RED LED " + i);
                redLEDs[redArrSetterIndex] = i;
                redArrSetterIndex++;
            }
            else {
                // System.out.println("THIS IS A BLUE LED " + i);
                blueLEDs[blueArrSetterIndex] = i;
                blueArrSetterIndex++;
            }
        }
    }
}

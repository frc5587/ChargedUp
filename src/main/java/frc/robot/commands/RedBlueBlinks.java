package frc.robot.commands;

import org.frc5587.lib.advanced.CustomLEDPattern;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class RedBlueBlinks implements CustomLEDPattern {
    private double stepTime = 0.02;
    private double idx, lastidx, speed = 0;
    private int brightness, length;
    private int[] redLEDs, blueLEDs;

    /**
     * @param speed how fast the pattern blinks, in 
     * @param length length of LED strip, in pixels
     * @param brightness from 0 to 255, its the brightness of the LED strip
     */
    public RedBlueBlinks(double speed, int length, int brightness) {
        this.speed = speed;
        this.brightness = brightness;
        this.length = length;
        
        updateLEDLists();
    }

    @Override
    public AddressableLEDBuffer step(int stepNumber, AddressableLEDBuffer ledBuffer) {
        if(idx > 2.1) {
            idx = 0;
        }

        // if(idx-lastidx > 0.5) {
            System.out.println(idx);
        // }
        
        for (int i = 0; i < redLEDs.length; i++) {
            if((int) idx == 0) {
                ledBuffer.setRGB(redLEDs[i], brightness, 0, 0);
            }
            else if((int) idx == 1) {
                ledBuffer.setRGB(redLEDs[i], 0, 0, brightness);
            }
        }
        for (int i = 0; i < blueLEDs.length; i++) {
            if((int) idx == 0) {
                ledBuffer.setRGB(blueLEDs[i], 0, 0, brightness);
            }
            else if((int) idx == 1) {
                ledBuffer.setRGB(blueLEDs[i], brightness, 0, 0);
            }
        }
        
        idx += stepTime * speed;

        return ledBuffer;
    }

    public void updateLEDLists() {
        redLEDs = new int[length/2+1];
        blueLEDs = new int[length/2+1];
        int redArrSetterIndex = 0;
        int blueArrSetterIndex = 0;
        for (int i = 0; i < length; i++) {
            if(((i <= length / 4)) || ((i <= length*0.75) && (i > length / 2))) {
                redLEDs[redArrSetterIndex] = i;
                redArrSetterIndex++;
            }
            else {
                blueLEDs[blueArrSetterIndex] = i;
                blueArrSetterIndex++;
            }
        }
    }
}

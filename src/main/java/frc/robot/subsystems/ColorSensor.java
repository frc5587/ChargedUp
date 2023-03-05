package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
    private final ColorSensorV3 sensor;
    private final ColorMatch colorMatcher = new ColorMatch();

    private final Color cubeColor = new Color(0, 0, 0); // TODO
    private final Color coneColor = new Color(0, 0, 0); // TODO

    public ColorSensor(I2C.Port port) {
        sensor = new ColorSensorV3(port);

        setName("Intake Color Sensor");

        colorMatcher.addColorMatch(cubeColor);
        colorMatcher.addColorMatch(coneColor);
    }

    public Color getClosestColor() {
        ColorMatchResult match = colorMatcher.matchClosestColor(sensor.getColor());

        return match.color;
    }

    public boolean hasCone() {
        if(getClosestColor() == coneColor) return true;

        return false;
    }

    public boolean hasCube() {
        if(getClosestColor() == cubeColor) return true;

        return false;
    }
}

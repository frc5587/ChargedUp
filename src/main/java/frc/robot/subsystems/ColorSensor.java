package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ColorSensor extends SubsystemBase {
    private final ColorSensorV3 sensor;
    private final ColorMatch colorMatcher = new ColorMatch();

    private final Color cubeColor = Color.kYellow;
    private final Color coneColor = Color.kPurple;

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
        return hasElementProximity() && getClosestColor() == coneColor;
    }

    public boolean hasCube() {
        return hasElementProximity() && getClosestColor() == cubeColor;
    }

    public boolean hasElementProximity() {
        return sensor.getProximity() < 150; // 15 cm?
    }

    public boolean hasElementColor() {
        return hasCone() || hasCube();
    }

    @Override
    public void periodic() {
        if(Robot.m_debugMode) {
            SmartDashboard.putString("ColorSensor ClosestColor", sensor.getColor().toString());
            SmartDashboard.putBoolean("Haselement", hasElementColor());
        }
    }
}
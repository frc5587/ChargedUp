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
    private final LEDs leds;
    private final Color cubeColor = new Color(62, 85, 105);//Color.kPurple;
    private final Color coneColor = new Color(118, 126, 9);//Color.kYellow;

    public ColorSensor(I2C.Port port, LEDs leds) {
        sensor = new ColorSensorV3(port);
        this.leds = leds;
        setName("Intake Color Sensor");

        colorMatcher.addColorMatch(cubeColor);
        colorMatcher.addColorMatch(coneColor);
        colorMatcher.addColorMatch(Color.kBlack);
        colorMatcher.addColorMatch(Color.kWhite);
        colorMatcher.setConfidenceThreshold(0.90);
    }

    public Color getClosestColor() {
        ColorMatchResult match = colorMatcher.matchClosestColor(sensor.getColor());

        return match.color;
    }

    public boolean hasCone() {
        return (hasElementProximity() && getClosestColor() == coneColor);
    }

    public boolean hasCube() {
        return (hasElementProximity() && getClosestColor() == cubeColor);
    }

    public boolean hasElementProximity() {
        return sensor.getProximity() > 200; // 15 cm?
    }

    public boolean hasElementColor() {
        return (hasCone() || hasCube());
    }

    @Override
    public void periodic() {
        if(Robot.m_debugMode) {
            SmartDashboard.putString("ColorSensor ClosestColor", getClosestColor().toString());
            SmartDashboard.putString("Color", sensor.getColor().toString());
            SmartDashboard.putBoolean("Has Cone", hasCone());
            SmartDashboard.putBoolean("Has Cube", hasCube());
            SmartDashboard.putBoolean("Haselement", hasElementColor());
            SmartDashboard.putNumber("Proximity", sensor.getProximity());
        }

        if(hasCone()) {
            leds.setYellow();
        }
        else if(hasCube()) {
            leds.setPurple();
        }
        else if(!leds.isRunningPurpleBlink && !leds.isRunningYellowBlink && !hasElementColor()){
            leds.setRainbow();
        }
    }
}
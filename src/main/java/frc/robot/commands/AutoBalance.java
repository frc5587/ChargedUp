package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
    private Swerve swerve;
    private LEDs leds;

    private Notifier blinkLeds = new Notifier(this::blinkLeds);

    private boolean isRedAlliance = DriverStation.getAlliance() == Alliance.Red;
    private double angleDegrees;

    private final double metersPerSec = 0.8;
    private final double positionThresholdDegrees = 6.5;

    public AutoBalance(Swerve swerve, LEDs leds) {
        this.swerve = swerve;
        this.leds = leds;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        blinkLeds.startPeriodic(0.4);

        angleDegrees = Double.POSITIVE_INFINITY;
    }

    @Override
    public void execute() {
        angleDegrees = swerve.getPitch();
        boolean shouldStop = Math.abs(angleDegrees) < positionThresholdDegrees;

        if(shouldStop) {
            System.out.println("Within angle threshold");
            swerve.stop();
            swerve.setChassisSpeeds(new ChassisSpeeds());
        } else {
            System.out.printf("Autobalancing %f%n", angleDegrees);
            swerve.setChassisSpeeds(new ChassisSpeeds(
                metersPerSec * (angleDegrees > 0.0 ? -metersPerSec : metersPerSec),
                0.0,
                0.0));   
        }
    }

    @Override
    public void end(boolean interrupted) {
        blinkLeds.stop();
        blinkLeds.close();
        leds.setAlliance();
    }

    boolean ledStatus = false;

    private void blinkLeds() {
        if(ledStatus == true) {
            leds.off();
            ledStatus = false;
        } 
        else {
            if(isRedAlliance) {
                leds.setColor(255, 0, 0); 
            }
            else {
                leds.setColor(0, 0, 255);
            }
            ledStatus = true;
        }
    }
}

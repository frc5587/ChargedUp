package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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

    private final double speedInchesPerSec = 8.5;
    private final double metersPerSec = Units.inchesToMeters(speedInchesPerSec);
    private final double positionThresholdDegrees = 3.0;
    private final double velocityThresholdDegreesPerSec = 8.0;

    public AutoBalance(Swerve swerve, LEDs leds) {
        this.swerve = swerve;
        this.leds = leds;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        blinkLeds.startPeriodic(0.4); // TODO Check

        angleDegrees = Double.POSITIVE_INFINITY;
    }

    @Override
    public void execute() {
        // angleDegrees = Math.sqrt(Math.pow(swerve.getPitch(), 2) + Math.pow(swerve.getRoll(), 2));
        angleDegrees = swerve.getPitch();
        // double angleVelocityDegreesPerSec = swerve.getRotation().getCos() * Units.radiansToDegrees(swerve.getPitchVelocity()) + swerve.getRotation().getSin() * Units.radiansToDegrees(swerve.getRollVelocity);
        // boolean shouldStop = (angleDegrees < 0.0 && angleVelocityDegreesPerSec > velocityThresholdDegreesPerSec.get()) || (angleDegrees > 0.0 && angleVelocityDegreesPerSec < -velocityThresholdDegreesPerSec.get());
        boolean shouldStop = Math.abs(angleDegrees) < 6.5;

        if(shouldStop) {
            System.out.println("Within angle threshold");
            swerve.stop();
            swerve.setChassisSpeeds(new ChassisSpeeds());
            // swerve.stopWithLock(true);
        } else {
            System.out.println("Autobalancing");
            swerve.setChassisSpeeds(new ChassisSpeeds(
                metersPerSec * (angleDegrees > 0.0 ? -0.8 : 0.8),
                0.0,
                0.0));

            // swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
            //     swerve.getPitch() / angleDegrees * metersPerSec,
            //     swerve.getRoll() / angleDegrees * metersPerSec,
            //     0.0,
            //     swerve.getYaw()));
            
        }
        System.out.println(angleDegrees);
    }

    @Override
    public void end(boolean interrupted) {
        // swerve.stopWithLock(true);
        blinkLeds.stop();
        blinkLeds.close();
        leds.setAlliance();
    }

    // @Override
    // public boolean isFinished() {
    //     return Math.abs(angleDegrees) < 6.5;
    // }

    // @Override
    // public boolean isFinished() {
    //     System.out.println(angleDegrees);
    //     return Math.abs(angleDegrees) < positionThresholdDegrees;
    // }

    boolean ledStatus = false;

    // TODO test this
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

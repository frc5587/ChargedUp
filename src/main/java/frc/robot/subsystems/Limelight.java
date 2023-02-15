package frc.robot.subsystems;

import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5587.lib.subsystems.LimelightBase;

public class Limelight extends LimelightBase {
    SendableChooser<LedValues> ledChooser = new SendableChooser<LedValues>();
    private final Field2d field = new Field2d();

    public Limelight() {
        super(LimelightConstants.mountAngle, LimelightConstants.lensHeight, LimelightConstants.goalHeight, LimelightConstants.distanceOffset);

        ledChooser.setDefaultOption("DEFAULT", LedValues.DEFAULT);
        ledChooser.addOption("ON", LedValues.ON);
        ledChooser.addOption("OFF", LedValues.OFF);
        ledChooser.addOption("BLINK", LedValues.BLINK);
        SmartDashboard.putData("Limelight LEDs", ledChooser);
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
        setLEDs(ledChooser.getSelected());
        double targetID = limelightTable.getEntry("tid").getDouble(-1.0);
        if(targetID != -1.0) {
            System.out.println(targetID);
        }
        else {
            System.out.println("No Target");
        }
        double[] limelightBotPose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        Pose2d robotPose = new Pose2d(limelightBotPose[0], limelightBotPose[1], Rotation2d.fromDegrees(limelightBotPose[5]));
        field.setRobotPose(robotPose);
    }
}

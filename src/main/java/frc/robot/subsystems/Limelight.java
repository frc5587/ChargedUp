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
        super(LimelightConstants.MOUNT_ANGLE, LimelightConstants.LENS_HEIGHT, LimelightConstants.GOAL_HEIGHT, LimelightConstants.DISTANCE_OFFSET);

        ledChooser.setDefaultOption("DEFAULT", LedValues.DEFAULT);
        ledChooser.addOption("ON", LedValues.ON);
        ledChooser.addOption("OFF", LedValues.OFF);
        ledChooser.addOption("BLINK", LedValues.BLINK);
        SmartDashboard.putData("Limelight LEDs", ledChooser);
        // SmartDashboard.putData("Limelight Pose Field", field);
    }

    public Pose2d getLimelightPose() {
        double[] limelightBotPose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        return new Pose2d(limelightBotPose[0], limelightBotPose[1], Rotation2d.fromDegrees(limelightBotPose[5]));
    }

    @Override
    public void periodic() {
        setLEDs(ledChooser.getSelected());
        // field.setRobotPose(getLimelightPose());
    }
}

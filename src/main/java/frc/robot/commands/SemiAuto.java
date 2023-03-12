package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoSetArm.GridHeight;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class SemiAuto {
    private final Swerve swerve;
    private final Arm arm;
    private final Intake intake;
    public int currentGridNumber;
    public Field2d desiredPoseField = new Field2d();

    public SemiAuto(Swerve swerve, Arm arm, Intake intake) {
        this.swerve = swerve;
        this.arm = arm;
        this.intake = intake;

        SmartDashboard.putData("Desired Pose Field", desiredPoseField);
    }

    public class DriveToGrid extends DriveToPose {
        /**
         * This uses the given gridNumber to select a pre-created {@link Pose2d} from constants.
         * The poses are numbered left to right from the driver's perspective on the blue side,
         * and right to left on the red side. A map is provided in the following image:
         * <img src="../util/doc-files/GridMap.png" width="100%" />
         */
        public DriveToGrid(int gridNumber) {
            super(swerve, AutoConstants.GRID_LOCATIONS[gridNumber].greaterPose);
            currentGridNumber = gridNumber;
            desiredPoseField.setRobotPose(AutoConstants.GRID_LOCATIONS[gridNumber].greaterPose);
        }
    }

    public class DriveWithinGrid extends DriveToPose {
        /** 
         * This uses the given posNumber to select a pre-created {@link Pose2d} from constants.
         * The poses are numbered left to right from the driver's perspective, 
         * where left is 0 and right is 2.
        */
        public DriveWithinGrid(int posNumber) {
            super(swerve, AutoConstants.GRID_LOCATIONS[currentGridNumber].poseArray[posNumber]);
            desiredPoseField.setRobotPose(AutoConstants.GRID_LOCATIONS[currentGridNumber].poseArray[posNumber]);
        }
    }

    public class ScoreInGrid extends SequentialCommandGroup {
        public ScoreInGrid(GridHeight height) {
            super(
                new AutoSetArm(arm, height),
                new DriveToPose(swerve, new Pose2d(swerve.getPose().getX()-0.2, swerve.getPose().getY(), new Rotation2d())),
                new WaitCommand(0.2), // This and following WaitCommands are arbitrary
                new InstantCommand(arm::lowerFromGrid),
                new InstantCommand(intake::backward),
                new WaitCommand(0.2),
                new InstantCommand(arm::liftAwayFromGrid),
                new DriveToPose(swerve, new Pose2d(swerve.getPose().getX()+0.2, swerve.getPose().getY(), new Rotation2d()))
            );
        }
    }

    public class ConeFlipper extends SequentialCommandGroup {
        public ConeFlipper() {
            super(
                new AutoSetArm(arm, GridHeight.Low),
                new ParallelCommandGroup(
                        new InstantCommand(intake::backward), 
                        new InstantCommand(() ->
                            swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                            -AutoConstants.CRAWL_SPEED,
                            0.0,
                            0.0,
                            swerve.getYaw()))))
            );
        }
    }
}

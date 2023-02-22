package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class SemiAuto {
    private final Swerve swerve;
    private final Arm arm;
    private final Intake intake;
    public final CommandSampler cSampler;
    public int currentGridNumber;

    public SemiAuto(Swerve swerve, Arm arm, Intake intake) {
        this.swerve = swerve;
        this.arm = arm;
        this.intake = intake;
        this.cSampler = new CommandSampler();
    }

    public class CommandSampler {
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
            }
        }
    }
}

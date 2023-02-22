package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
        public class DriveToGrid extends CommandBase {
            int gridNumber;
            Swerve swerve;
            DriveToPose driveToPose;
    
            /** 
             * This image denotes how the grids are numbered:
             * <img src="../util/doc-files/GridMap.png" width=10 />
             * and
             */
            public DriveToGrid(int gridNumber, Swerve swerve) {
                this.gridNumber = gridNumber;
                this.swerve = swerve;
                driveToPose = new DriveToPose(swerve, AutoConstants.GRID_LOCATIONS[gridNumber].greaterPose);
                currentGridNumber = gridNumber;
            }

            @Override
            public void initialize() {
                driveToPose.initialize();
            }

            @Override
            public void execute() {
                driveToPose.execute();
            }

            @Override
            public void end(boolean interrupted) {
                driveToPose.end(interrupted);
            }
        }

        public class DriveWithinGrid extends CommandBase {
            int posNumber;
            Swerve swerve;
            DriveToPose driveToPose;
    
            /** 
             * This image denotes how the positions within grids are numbered:
             * <img src="../util/doc-files/MapWithinGrid.png" width="10px" />
             */
            public DriveWithinGrid(int posNumber, Swerve swerve) {
                this.posNumber = posNumber;
                this.swerve = swerve;
                switch (posNumber) {
                    case 0:
                        driveToPose = new DriveToPose(swerve, AutoConstants.GRID_LOCATIONS[currentGridNumber].poseLeft);
                        break;
                
                    case 1:
                        driveToPose = new DriveToPose(swerve, AutoConstants.GRID_LOCATIONS[currentGridNumber].greaterPose);
                        break;

                    case 2:
                        driveToPose = new DriveToPose(swerve, AutoConstants.GRID_LOCATIONS[currentGridNumber].poseRight);
                        break;

                    default:
                        break;
                }
                
            }

            @Override
            public void initialize() {
                driveToPose.initialize();
            }

            @Override
            public void execute() {
                driveToPose.execute();
            }

            @Override
            public void end(boolean interrupted) {
                driveToPose.end(interrupted);
            }
        }
    }
}

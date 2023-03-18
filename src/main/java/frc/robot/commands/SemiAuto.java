package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoSetArm.GridHeight;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;

public class SemiAuto {
    private final Swerve swerve;
    private final Arm arm;
    private final Intake intake;
    private final LEDs leds;
    public int currentGridNumber;
    // public Field2d desiredPoseField = new Field2d();

    public enum SubstationType {
        DoubleFar(0),
        DoubleBump(1),
        Single(2);

        int value;
        SubstationType(int value) {
            this.value = value;
        }
    }

    public SemiAuto(Swerve swerve, Arm arm, Intake intake, LEDs leds) {
        this.swerve = swerve;
        this.arm = arm;
        this.intake = intake;
        this.leds = leds;

        // SmartDashboard.putData("Desired Pose Field", desiredPoseField);
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
            // desiredPoseField.setRobotPose(AutoConstants.GRID_LOCATIONS[gridNumber].greaterPose);
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
            // desiredPoseField.setRobotPose(AutoConstants.GRID_LOCATIONS[currentGridNumber].poseArray[posNumber]);
        }
    }

    public class ScoreInGrid extends SequentialCommandGroup {
        public ScoreInGrid(GridHeight height) {
            super(
                // new DriveToPose(swerve, new Pose2d(
                //         DriverStation.getAlliance() == Alliance.Blue ? swerve.getPose().getX()+0.57 : swerve.getPose().getX()-0.57, swerve.getPose().getY(), swerve.getPose().getRotation())),
                new ParallelDeadlineGroup(
                        new WaitCommand(1), 
                        new RunCommand(() -> swerve.crawl(DriverStation.getAlliance() == Alliance.Blue ? -0.7 : 0.7), swerve)),
                new AutoSetArm(arm, height),
                new InstantCommand(arm::liftAwayFromGrid),
                // new DriveToPose(swerve, new Pose2d(
                //         DriverStation.getAlliance() == Alliance.Blue ? swerve.getPose().getX()-0.57 : swerve.getPose().getX()+0.57, swerve.getPose().getY(), swerve.getPose().getRotation())),
                new WaitCommand(1.2),
                new ParallelDeadlineGroup(
                        new WaitCommand(1), 
                        new RunCommand(() -> swerve.crawl(DriverStation.getAlliance() == Alliance.Blue ? 0.7 : -0.7), swerve)),
                new WaitCommand(0.5), // This and following WaitCommands are arbitrary
                new InstantCommand(arm::lowerFromGrid),
                new InstantCommand(intake::backward),
                new WaitCommand(0.2),
                new InstantCommand(arm::liftAwayFromGrid),
                // new DriveToPose(swerve, new Pose2d(DriverStation.getAlliance() == Alliance.Blue ? swerve.getPose().getX()+0.57 : swerve.getPose().getX()-0.57, swerve.getPose().getY(), swerve.getPose().getRotation()))
                new ParallelDeadlineGroup(
                        new WaitCommand(1), 
                        new RunCommand(() -> swerve.crawl(DriverStation.getAlliance() == Alliance.Blue ? -0.7 : 0.7), swerve)),
                new AutoSetArm(arm, GridHeight.Intake)
            );
            addRequirements(swerve);
        }
    }

    public class DriveToSubstation extends SequentialCommandGroup {
        public DriveToSubstation(SubstationType sub) {
            super(
                new ParallelCommandGroup(
                        new DriveToPose(
                            swerve, DriverStation.getAlliance() == Alliance.Red
                                ? AutoConstants.RED_SUBS[sub.value]
                                : AutoConstants.BLUE_SUBS[sub.value]),
                        new AutoSetArm(arm, GridHeight.Substation)),
                new ParallelDeadlineGroup(
                        new IntakeIn(intake),
                        new RunCommand(() -> swerve.crawl(0), swerve)),
                new ParallelDeadlineGroup(
                        new WaitCommand(1), 
                        new RunCommand(() -> swerve.crawl(.5))),
                new InstantCommand(() -> leds.setColor(0, 70, 0))
            );
            addRequirements(swerve, intake);
        }
    }

    public class ConeFlipper extends SequentialCommandGroup {
        public ConeFlipper() {
            super(
                new AutoSetArm(arm, GridHeight.Low),
                new ParallelRaceGroup(
                        new InstantCommand(intake::backward), 
                        new RepeatCommand(new InstantCommand(() ->
                            swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                            -AutoConstants.CRAWL_SPEED,
                            0.0,
                            0.0,
                            swerve.getYaw())))),
                            new WaitCommand(1)), //TODO CHANGE THIS!!!!!!
                new InstantCommand(swerve::stop)
            );
        }
    }
}

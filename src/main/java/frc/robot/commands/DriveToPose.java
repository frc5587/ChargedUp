package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.Supplier;

public class DriveToPose extends CommandBase {
    private final Swerve swerve;
    private final Supplier<Pose2d> poseSupplier;

  private boolean running = false;

  /** Drives to the specified pose under full software control. */
  public DriveToPose(Swerve swerve, Pose2d pose) {
    this(swerve, () -> pose);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(Swerve swerve, Supplier<Pose2d> poseSupplier) {
    this.swerve = swerve;
    this.poseSupplier = poseSupplier;
    addRequirements(swerve);
    AutoConstants.BOT_ANGLE_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    // Reset all controllers
    var currentPose = swerve.getPose();
    AutoConstants.BOT_DRIVE_CONTROLLER.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()));
    AutoConstants.BOT_ANGLE_CONTROLLER.reset(currentPose.getRotation().getRadians());
  }

    @Override
    public void execute() {
        this.running = true;
        // Get current and target pose
        Pose2d currentPose = swerve.getEstimatedPose();
        Pose2d targetPose = poseSupplier.get();

        // Command speeds
        double driveVelocityScalar =
            AutoConstants.BOT_DRIVE_CONTROLLER.calculate(
                currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()), 0.0);
        double thetaVelocity =
            AutoConstants.BOT_ANGLE_CONTROLLER.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        if (AutoConstants.BOT_DRIVE_CONTROLLER.atGoal()) driveVelocityScalar = 0.0;
        if (AutoConstants.BOT_ANGLE_CONTROLLER.atGoal()) thetaVelocity = 0.0;
        var driveVelocity =
            new Pose2d(
                    new Translation2d(),
                    currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(new Transform2d(new Translation2d(driveVelocityScalar, 0.0), new Rotation2d()))
                .getTranslation();
        swerve.setChassisSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    this.running = false;
    swerve.stop();
  }

  public boolean atGoal() {
    return running && AutoConstants.BOT_DRIVE_CONTROLLER.atGoal() && AutoConstants.BOT_ANGLE_CONTROLLER.atGoal();
  }
}
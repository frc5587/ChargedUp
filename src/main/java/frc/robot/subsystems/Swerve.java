package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;
    public SwerveDriveOdometry odometry;
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveDriveKinematics kinematics;
    public TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(1.5); 
    public Limelight limelight;
    public Field2d field = new Field2d();

    public Swerve(Limelight limelight) {
        this.gyro = new AHRS();
        zeroGyro();
        this.limelight = limelight;

        this.mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.Mod0.MODULECONSTANTS),
            new SwerveModule(1, SwerveConstants.Mod1.MODULECONSTANTS),
            new SwerveModule(2, SwerveConstants.Mod2.MODULECONSTANTS),
            new SwerveModule(3, SwerveConstants.Mod3.MODULECONSTANTS)
        };

        this.odometry = new SwerveDriveOdometry(SwerveConstants.SWERVE_KINEMATICS, getYaw(), getModulePositions());

        this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, getYaw(), getModulePositions(), getPose(), // ! these numbers are 100% not tuned
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1), // State measurement standard deviations. X, Y, theta.
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.03, 0.03, 0.2)); // Vision standard deviations.

        SmartDashboard.putData("Swerve Pose Field", field);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            SwerveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        gyro.zeroYaw();
        gyro.setAngleAdjustment(pose.getRotation().getDegrees());

        odometry.resetPosition(getYaw(), getModulePositions(), pose);
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void zeroOdometry() {
        resetOdometry(new Pose2d());
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
        return (SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }



    @Override
    public void periodic(){
        odometry.update(getYaw(), getModulePositions());  
        poseEstimator.update(getYaw(), getModulePositions());
        // if the target is within 2.5 meters of the robot, factor it into pose data
        if(limelight.hasTarget() && limelight.calculateDistance() < 2.5) { // TODO: tune distance requirement
            poseEstimator.addVisionMeasurement(limelight.getLimelightPose(), Timer.getFPGATimestamp());
        }
        poseHistory.addSample(Timer.getFPGATimestamp(), getPose());
        field.setRobotPose(poseEstimator.getEstimatedPosition());

        if (DriverStation.isDisabled()){
            resetModulesToAbsolute();
        }

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANDegrees", mod.getCanCoder().getDegrees()-mod.angleOffset.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANAbsolute", mod.angleEncoder.getAbsolutePosition());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Falcon", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import frc.robot.util.swervelib.math.Conversions;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;
    public SwerveDriveOdometry odometry;
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveDriveKinematics kinematics;
    public TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(1.5); 
    public Limelight limelight;
    public Field2d field = new Field2d();
    private boolean locked = false;
    private SlewRateLimiter slew = new SlewRateLimiter(SwerveConstants.SLEW_RATE);

    public Swerve(Limelight limelight) {
        this.gyro = new AHRS();
        zeroGyro();
        this.limelight = limelight;
        this.kinematics = SwerveConstants.SWERVE_KINEMATICS;

        this.mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.Mod0.MODULECONSTANTS),
            new SwerveModule(1, SwerveConstants.Mod1.MODULECONSTANTS),
            new SwerveModule(2, SwerveConstants.Mod2.MODULECONSTANTS),
            new SwerveModule(3, SwerveConstants.Mod3.MODULECONSTANTS)
        };

        this.odometry = new SwerveDriveOdometry(SwerveConstants.SWERVE_KINEMATICS, getYaw(), getModulePositions());

        this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, getYaw(), getModulePositions(), getPose(), // ! these numbers are 100% not tuned
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, 0.01), // State measurement standard deviations. X, Y, theta.
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1)); // Vision standard deviations.

        SmartDashboard.putData("Swerve Pose Field", field);
        SmartDashboard.putBoolean("SWERVE BRAKE MODE", true);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // if(modsStopped()) {
        //     setModuleStates(new SwerveModuleState[] {
        //         new SwerveModuleState(0, new Rotation2d(45)),
        //         new SwerveModuleState(0, new Rotation2d(-45)),
        //         new SwerveModuleState(0, new Rotation2d(-45)),
        //         new SwerveModuleState(0, new Rotation2d(45)),
        //     });
        //     System.out.println("LOCKING");
        // }
        // else {
            SwerveModuleState[] swerveModuleStates =
                SwerveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                    fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                            translation.getX(), //TODO invert getX??????
                            translation.getY(), 
                            rotation, 
                            getYaw())
                        : new ChassisSpeeds(
                            -translation.getX(), //TODO invert getX????
                            translation.getY(), 
                            rotation));
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED);
            for(SwerveModule mod : mSwerveMods){
                mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            }
        // }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
        // return getEstimatedPose();
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
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
        //TODO invert states??????
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
        return (SwerveConstants.INVERT_GYRO) ? Rotation2d.fromDegrees(-gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void stop() {
        setChassisSpeeds(new ChassisSpeeds());
    }


    public void stopWithLock(boolean lock) {
        this.locked = lock;
        if(lock) {
            stop();
        }
    }

    private boolean modsStopped() {
        int stoppedModules = 0;
        for(SwerveModuleState state : getModuleStates()) {
            if(state.speedMetersPerSecond < 0.1) {
                stoppedModules ++;
            }
        }
        return stoppedModules == 4;
    }

    @Override
    public void periodic(){
        odometry.update(getYaw(), getModulePositions());  
        poseEstimator.update(getYaw(), getModulePositions());
        // SmartDashboard.putNumber("roll", getRoll());
        // SmartDashboard.putNumber("pitch", getPitch());
        // SmartDashboard.putNumber("yaw", getYaw().getDegrees());
        for(int i = 0; i < mSwerveMods.length; i++) {
            SmartDashboard.putNumber("mod " + i + "degrees", mSwerveMods[i].getCanCoder().getDegrees());
            SmartDashboard.putNumber("Adjusted " + i, mSwerveMods[i].getPosition().angle.getDegrees());
        }
        // if the target is within 2.5 meters of the robot, factor it into pose data
        if(limelight.hasTarget() && limelight.calculateDistance() < 2.5) { // TODO: tune distance requirement
            poseEstimator.addVisionMeasurement(limelight.getLimelightPose(), Timer.getFPGATimestamp());
        }
        poseHistory.addSample(Timer.getFPGATimestamp(), getPose());
        field.setRobotPose(poseEstimator.getEstimatedPosition());

        int stoppedModuleCounter = 0;
        for(SwerveModule module : mSwerveMods) {
            double velocity = Conversions.falconToMPS(module.mDriveMotor.getSelectedSensorVelocity(), SwerveConstants.CHOSEN_MODULE.wheelCircumference, SwerveConstants.CHOSEN_MODULE.driveGearRatio);
            if(velocity < .5) {
                stoppedModuleCounter++;
            }
            System.out.println(velocity);
        }

        // if(stoppedModuleCounter == 4) {
        //     stopWithLock(true);
        //     System.out.println("ALLMODSLOW");
        // }
        if (DriverStation.isDisabled()){
            resetModulesToAbsolute();
        }

        if(!SmartDashboard.getBoolean("SWERVE BREAK MODE", true)) {
            for(SwerveModule mod : mSwerveMods) {
                mod.mAngleMotor.setNeutralMode(NeutralMode.Coast);
                mod.mDriveMotor.setNeutralMode(NeutralMode.Coast);
            }
        }
        else {
            for(SwerveModule mod : mSwerveMods) {
                mod.mAngleMotor.setNeutralMode(NeutralMode.Brake);
                mod.mDriveMotor.setNeutralMode(NeutralMode.Brake);
            }
        }
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.USE_VISION;
import static frc.robot.Constants.LoggingConstants.kSwerveTab;
import static frc.robot.Constants.PathPlannerConstants.kPPRotationPIDConstants;
import static frc.robot.Constants.PathPlannerConstants.kPPTranslationPIDConstants;
import static frc.robot.Constants.SwerveDriveConstants.kApplyRobotSpeedsRequest;
import static frc.robot.Constants.SwerveDriveConstants.kFieldOrientedSwerveRequest;
import static frc.robot.Constants.SwerveDriveConstants.kRobotOrientedSwerveRequest;
import static frc.robot.Constants.SwerveDriveConstants.kTargetDriveController;
import static frc.robot.Constants.SwerveDriveConstants.kTargetDriveMaxLateralVelocity;
import static frc.robot.Constants.SwerveDriveConstants.kTowSwerveRequest;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.util.sendable.Sendable;
import org.wpilib.util.sendable.SendableBuilder;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.NerdyMath;
import frc.robot.util.logging.NerdLog;
import frc.robot.util.logging.Reportable;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

public class NerdDrivetrain extends TunerSwerveDrivetrain implements Subsystem, Reportable, Sendable {
    public final Field2d field;
    public boolean useMegaTag2 = false;

    public NerdDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);

        RobotConfig robotConfig = null;
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getChassisVelocities,
            (speeds, feedforwards) -> setControl(
                kApplyRobotSpeedsRequest.withVelocity(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
            new PPHolonomicDriveController(
                kPPTranslationPIDConstants, 
                kPPRotationPIDConstants),  
            robotConfig,
            () -> {
                var alliance = MatchState.getAlliance();
                return alliance.isPresent() ? (alliance.get() == Alliance.RED) : false;
            },
            this
        );

        field = new Field2d();

        setVision(USE_VISION);
    }
    
    
    @Override
    public void periodic() {
        field.setRobotPose(getPose());

        if (USE_VISION) {
            // visionUpdate(Camera.Example);
            visionUpdate(Camera.Front, true);
            visionUpdate(Camera.Back, RobotState.isTeleop());
        }
    }

    // ----------------------------------------- Drive Functions ----------------------------------------- //

    /**
     * drive in field oriented
     * @param xSpeed - how fast in the +x direction in m/s
     * @param ySpeed - how fast in the +y direction in m/s
     * @param rSpeed - how fast in the CCW direction in rad/s
     */
    public void driveFieldOriented(double xSpeed, double ySpeed, double rSpeed) {
        setControl(kFieldOrientedSwerveRequest
            .withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(rSpeed)
        );
    }

    /**
     * drive in robot oriented
     * @param xSpeed - how fast in the +x direction (forward) in m/s
     * @param ySpeed - how fast in the +y direction (left) in m/s
     * @param rSpeed - how fast in the CCW direction in rad/s
     */
    public void driveRobotOriented(double xSpeed, double ySpeed, double rSpeed) {
        setControl(kRobotOrientedSwerveRequest
            .withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(rSpeed)
        );
    }

    /**
     * drive to a target in blue oriented space, must be called continously at 50 Hz
     * call {@link #resetTargetDrive()} when starting
     * @param target - point in blue oriented space
     */
    public void driveToTarget(Pose2d target) {
        // since the outputs are also capped, there is a limit to the influence of one axis on the direction
        // this can lead to larger angles, maybe saving on necessary precision?
        // DriverStation.reportWarning(controller.getConstraints().maxVelocity + " " + controller.getConstraints().maxAcceleration, false);
        double x = kTargetDriveController.calculate("x", getPose().getX(), target.getX());
        double y = kTargetDriveController.calculate("y", getPose().getY(), target.getY());
        double r = kTargetDriveController.calculate("r", getSwerveHeadingRadians(), MathUtil.inputModulus(target.getRotation().getRadians(), -Math.PI, Math.PI));
        double l = Math.sqrt(x*x+y*y);
        // clamp the velocity
        x *= Math.min(1.0, kTargetDriveMaxLateralVelocity / l);
        y *= Math.min(1.0, kTargetDriveMaxLateralVelocity / l);
        if (kTargetDriveController.atSetpoint("x")) x = 0.0;
        if (kTargetDriveController.atSetpoint("y")) y = 0.0;
        if (kTargetDriveController.atSetpoint("r")) r = 0.0;
        // DriverStation.reportWarning(x + " " + y + " " + r, false);

        driveFieldOriented(x, y, r);
    }

    /**
     * resets the target drive controller for {@link #driveToTarget(Pose2d)}
     */
    public void resetTargetDrive() {
        kTargetDriveController.reset("x", getPose().getX(), getFieldOrientedVelocities().vx * 0.1);
        kTargetDriveController.reset("y", getPose().getY(), getFieldOrientedVelocities().vy * 0.1);
        kTargetDriveController.reset("r", getSwerveHeadingRadians(), getFieldOrientedVelocities().omega * 0.1);
    }

    // ----------------------------------------- Helper Functions ----------------------------------------- //

    public void tow() {
        setControl(kTowSwerveRequest);
    }

    public void stop() {
        driveFieldOriented(0.0, 0.0, 0.0);
    }

    /** gets the Pose2d from odometry */
    public Pose2d getPose() {
        return getState().Pose;
    }

    /** the position we will be one step in time */
    public Pose2d getLookAheadPose(double factor) {
        ChassisVelocities speeds = getFieldOrientedVelocities();
        return getPose().plus(new Transform2d(speeds.vx, speeds.vy, Rotation2d.kZero).times(factor));
    }

    /** gets the ChassisSpeeds from odometry */
    public ChassisVelocities getChassisVelocities() {
        return getState().Velocity;
    }

    public ChassisVelocities getFieldOrientedVelocities() {
        return getChassisVelocities();
    }

    public void setBrake(boolean brake) {
        configNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public double angleToPose(FieldPositions pos) {
        return NerdyMath.angleToPose(getPose(), pos.get());
    }

    public double angleToLookAheadPose(FieldPositions pos, double factor) {
        return NerdyMath.angleToPose(getLookAheadPose(factor), pos.get());
    }

    private final double deviceTempThreshold = 50;
    /** @return "it's chill" unless the temperature of any motor is above the threshold, reports id and type */
    public String pollTemperatures() {
        String output = "";
        for (int i = 0; i < 4; i++) {
            double driveTemp = getModule(i).getDriveMotor().getDeviceTemp().getValueAsDouble();
            double steerTemp = getModule(i).getSteerMotor().getDeviceTemp().getValueAsDouble();
            if (driveTemp >= deviceTempThreshold) output += "Drive " + i + ", Temp: " + driveTemp;
            if (steerTemp >= deviceTempThreshold) output += "Steer " + i + ", Temp: " + steerTemp;
            
        }
        if (!output.equals("")) return output;
        return "it's chill";
    }

    private double pollStatorCurrentSum() {
        double sum = 0;
        for (int i = 0; i < 4; i++) {
            sum += Math.abs(getModule(i).getDriveMotor().getTorqueCurrent().getValueAsDouble());
            sum += Math.abs(getModule(i).getSteerMotor().getTorqueCurrent().getValueAsDouble());
        }
        return sum;
    }

    // ----------------------------------------- Vision Functions ----------------------------------------- //

    /**
     * activates or deactivates vision by setting the pipeline either to 0 for active or 1 for inactive
     * and by adjusting throttle, see {@link LimelightHelpers#SetThrottle(String, int)}
     * @param activate whether to activate or deactivate
     */
    public void setVision(boolean activate) {
        for (Camera camera : Camera.values()) {
            LimelightHelpers.setPipelineIndex(camera.name, (activate) ? 0 : 0);
            LimelightHelpers.SetThrottle(camera.name, (activate) ? 0 : 0);
        }
    }
    
    /**
     * temporarily switch to megatag1 to update robot field heading/pose
     * @param delay in seconds until switching back to megatag2
     */
    public Command resetPoseWithAprilTags(double delay) {
        return Commands.sequence(
            Commands.runOnce(() -> useMegaTag2 = false),
            Commands.waitSeconds(delay),
            Commands.runOnce(() -> useMegaTag2 = true)
      );
    }

    // private HashMap<Camera, Double> lastTimestamps = new HashMap<>();    
    public void visionUpdate(Camera limelight, boolean useReset) {
        if (LimelightHelpers.getCurrentPipelineIndex(limelight.name) != 0) return;
        if (!useMegaTag2) {
            // --------- MT1 --------- //
            if (!useReset) return;
            PoseEstimate mt = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.name);
            if (mt == null || Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720 || mt.tagCount == 0 || mt.avgTagDist >= 3.0) return;
            resetRotation(mt.pose.getRotation());
            useMegaTag2 = true;
            setDriverHeadingForward();
        }
        else {
            // --------- MT2 --------- //
            double yaw = getSwerveHeadingDegrees();
            LimelightHelpers.SetRobotOrientation(limelight.name, yaw, 0, 0, 0, 0, 0);
            PoseEstimate mt = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.name);
            field.getObject(limelight.name).setPose(nullPose);
            if (mt == null || Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720 || mt.tagCount == 0) return;
            // if (!lastTimestamps.containsKey(limelight)) lastTimestamps.put(limelight, 0.0);
            // if (lastTimestamps.get(limelight).equals(mt.timestampSeconds)) return;
            // lastTimestamps.put(limelight, mt.timestampSeconds);
            field.getObject(limelight.name).setPose(mt.pose);
            double stddev = (mt.avgTagDist > 3) ? 2.0 : 0.7;
            setVisionMeasurementStdDevs(VecBuilder.fill(stddev, stddev, 9999999)); // TODO consider other stddevs
            addVisionMeasurement(mt.pose, Utils.getCurrentTimeSeconds());
        }
    }
    private Pose2d nullPose = new Pose2d(-100,-100, Rotation2d.kZero);

    public void recalibrateGyroMT1() {
        resetRotation((RobotContainer.IsRedSide()) ? Rotation2d.k180deg : Rotation2d.kZero);
        useMegaTag2 = false;
    }

    // ----------------------------------------- Gyro Functions ----------------------------------------- //

    /** 
     * Set the operator heading to forward based on alliance field forward
     * @see {@link #setOperatorPerspectiveForward} also for more custom setting
     */
    public void setDriverHeadingForward() {
        setOperatorPerspectiveForward(RobotContainer.IsRedSide() ? Rotation2d.k180deg : Rotation2d.kZero);
    }

    /** 
     * Set the operator heading to forward based on robot
     * @see {@link #setOperatorPerspectiveForward} also for more custom setting
     */
    public void setRobotHeadingForward() {
        setOperatorPerspectiveForward(getPose().getRotation());
    }
    
    /**
     * Get heading relative to what the operator sees in degrees
     * @see {@link #setDriverHeadingForward()} for resetting to zero
     */
    public double getDriverHeadingDegrees() {
        return getOperatorForwardDirection().getDegrees() + getSwerveHeadingDegrees();
    }

    /**
     * Get heading relative to what the operator sees in radians
     * @see {@link #setDriverHeadingForward()} for resetting to zero
     */
    public double getDriverHeadingRadians() {
        return getOperatorForwardDirection().getRadians() + getSwerveHeadingRadians();
    }

    /** 
     * Get absolute heading in degrees, from blue alliance orientation
     * @see {@link #resetRotation(Rotation2d)}
     */
    public double getSwerveHeadingDegrees() {
        return MathUtil.inputModulus(getPose().getRotation().getDegrees(), -180, 180);
    }

    /** 
     * Get absolute heading in radians, from blue alliance orientation
     * @see {@link #resetRotation(Rotation2d)}
     */
    public double getSwerveHeadingRadians() {
        return MathUtil.inputModulus(getPose().getRotation().getRadians(), -Math.PI, Math.PI);
    }

    // ----------------------------------------- Logging Functions ----------------------------------------- //

    @Override
    public void initializeLogging() {
        NerdLog.logData(kSwerveTab + "/Robot Field", field, LOG_LEVEL.MINIMAL);

        ///////////
        /// ALL ///
        ///////////
        NerdLog.logData(kSwerveTab + "/Commands", this, LOG_LEVEL.ALL);
        if (Constants.ROBOT_LOG_LEVEL == LOG_LEVEL.ALL) {
            Field2d positionField = new Field2d();
            for (FieldPositions position : FieldPositions.values()) {
                positionField.getObject(position.name() + "-blue").setPose(position.blue);
                positionField.getObject(position.name() + "-red").setPose(position.red);
            }
            NerdLog.logData(kSwerveTab +"/Object Field", positionField, LOG_LEVEL.ALL);
        }
        for (Camera camera : Camera.values())
            NerdLog.logBoolean(kSwerveTab + "/" + camera.name + " detecting", () -> LimelightHelpers.getTV(camera.name), LOG_LEVEL.ALL);

        NerdLog.logStructSerializable(kSwerveTab + "/Field Chassis Speeds", () -> getFieldOrientedVelocities(), LOG_LEVEL.ALL);
        NerdLog.logSwerveModules(kSwerveTab + "/Swerve Module States", this::getState, LOG_LEVEL.ALL);

        //////////////
        /// MEDIUM ///
        //////////////
        NerdLog.logNumber(kSwerveTab + "/Swerve Heading", this::getSwerveHeadingDegrees, "deg", LOG_LEVEL.MEDIUM);
        NerdLog.logNumber(kSwerveTab + "/Driver Heading", this::getDriverHeadingDegrees, "deg", LOG_LEVEL.MEDIUM);
        NerdLog.logBoolean(kSwerveTab + "/Using MT2", () -> this.useMegaTag2, LOG_LEVEL.MEDIUM);
        
        //////////////
        /// MINIMAL //
        //////////////
        for (int i = 0; i < 4; i++) {
            NerdLog.logSignal(kSwerveTab + "/Temperatures/Drive " + i, getModule(i).getDriveMotor().getDeviceTemp(false), getModule(i).getDriveMotor().getNetwork().getName(), LOG_LEVEL.MINIMAL);
            NerdLog.logSignal(kSwerveTab + "/Temperatures/Turn " + i, getModule(i).getSteerMotor().getDeviceTemp(false), getModule(i).getSteerMotor().getNetwork().getName(), LOG_LEVEL.MINIMAL);
            NerdLog.logBoolean(kSwerveTab + "/Connected/Drive " + i, getModule(i).getDriveMotor()::isConnected, LOG_LEVEL.MINIMAL);
            NerdLog.logBoolean(kSwerveTab + "/Connected/Turn " + i, getModule(i).getSteerMotor()::isConnected, LOG_LEVEL.MINIMAL);
        }
        NerdLog.logNumber(kSwerveTab +"/Stator Current Sum", this::pollStatorCurrentSum, "A", LOG_LEVEL.MINIMAL);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Subsystem");

        builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);
        builder.addStringProperty(
            ".default",
            () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none",
            null);
        builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null, null);
        builder.addStringProperty(
            ".command",
            () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none",
            null);
  }
}

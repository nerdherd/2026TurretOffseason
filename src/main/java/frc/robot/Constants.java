// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot;
 
import java.util.function.BiFunction;
import java.util.function.Function;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.signals.*;
import com.limelightvision.Limelight;
import com.limelightvision.PoseEstimateConfig;
import com.pathplanner.lib.config.PIDConstants;

import frc.robot.Constants.SwerveDriveConstants.FieldPositions;
import frc.robot.subsystems.TurretSwivel.TurretSwivel;
import frc.robot.subsystems.TurretSwivel.TurretSwivelConfiguration;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.trajectory.TrapezoidProfile.Constraints;
import org.wpilib.math.util.Units;

import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.subsystems.template.TemplateSubsystem.SubsystemMode;
import frc.robot.util.nerd_constants.NerdPIDConstants;
import frc.robot.util.nerd_logging.Reportable.LOG_LEVEL;
import frc.robot.util.nerd_math.LInTable;
import frc.robot.util.nerd_math.MultiProfiledPIDController;
import frc.robot.util.nerd_math.NerdFlippingUtil;
import frc.robot.util.nerd_math.NerdyMath;
import frc.robot.util.nerd_math.Translation2dSlewRateLimiter;
import frc.robot.util.nerd_zones.NerdZone;
import frc.robot.util.nerd_zones.RectangleZone;
import frc.robot.util.nerd_zones.ZoneGroup;
 
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
 
 // COMMENT ROBOT IDS INSTEAD OF DELETING
 
public final class Constants {
 
  /** current logging level of the robot's subsystems, @see Reportable.add... */
  public static final LOG_LEVEL ROBOT_LOG_LEVEL = LOG_LEVEL.ALL;
  
  /** 
   * (hopefully) controls whether subsystem objects are used, swerve and others not counted
   * @see {@link frc.robot.subsystems.template.TemplateSubsystem TemplateSubsystem} 
   * @see {@link frc.robot.subsystems.SuperSystem SuperSystem}
   */
  public static final boolean USE_SUBSYSTEMS = true;
  /**
   * controls whether vision should be initialized
   */
  public static final boolean USE_VISION = true;
 
  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kTestControllerPort = 2;
 
    // deadbands
    public static final double kTranslationDeadband = 0.1; // out of 1
    public static final double kRotationDeadband = 0.1; // out of 1
    public static final double kTurnToAngleDeadband = 0.5; // out of 1
 
    public static final double kInputAcceleration = 1.0; // unit/s, on the scale of a unit circle/fractions
    public static final double kEasePower = 3.0; // increase to further separate lower and higher values
 
    public static final Translation2dSlewRateLimiter kTranslationInputRateLimiter = new Translation2dSlewRateLimiter(kInputAcceleration);
    
    // returns a vector within the unit circle
    public static final BiFunction<Double, Double, Translation2d> kTranslationInputFilter = 
    (x, y) -> {
        x = NerdyMath.deadband(x, kTranslationDeadband);
        y = NerdyMath.deadband(y, kTranslationDeadband);
        if (x == 0.0 && y == 0.0) {
          kTranslationInputRateLimiter.reset();
          return Translation2d.ZERO;
        }
        Translation2d dir = new Translation2d(x, y);
        double length = dir.getNorm();
        dir = dir.div(length);
        length = Math.min(1.0, length);
        length = Math.pow(length, kEasePower);
        return kTranslationInputRateLimiter.calculate(dir.times(length));
    };
 
    public static final Function<Double, Double> kRotationInputFilter = 
    (r) -> {
      return NerdyMath.deadband(r, kRotationDeadband);
    };
 
    public static final BiFunction<Double, Double, Double> kTurnToAngleFilter =
    (x, y) -> {
      if (NerdyMath.isPoseInsideCircleZone(0.0, 0.0, kTurnToAngleDeadband, x, y)) return Double.NaN;
      return Math.atan2(y, x);
    };
  }

  public static final class ChoreoConstants {
    private final static NerdPIDConstants kXControllerConstants = new NerdPIDConstants(10.0, 0.0, 0);
    private final static NerdPIDConstants kYControllerConstants = new NerdPIDConstants(10.0, 0.0, 0);
    private final static NerdPIDConstants kRControllerConstants = new NerdPIDConstants(7.5, 10.0, 0).enableContinuousInput(-Math.PI, Math.PI);

    public final static PIDController kXController = kXControllerConstants.getController();
    public final static PIDController kYController = kYControllerConstants.getController();
    public final static PIDController kRController = kRControllerConstants.getController();
  }


  public static final class PathPlannerConstants {
    public static final PIDConstants kPPTranslationPIDConstants = new PIDConstants(5, 0, 0);
    public static final PIDConstants kPPRotationPIDConstants = new PIDConstants(5, 0, 0);
  }

  public static final class LoggingConstants {
    public static final double LOGGING_INTERVAL = 0.02; // seconds

    public static final String kSubsystemTab = "SuperSystem/"; // ends with a /
    public static final String kSupersystemTab = "SuperSystem";
    public static final String kSwerveTab = "SwerveDrive";
    public static final String kAutosTab = "Autos";
  }

  public static final class VisionConstants {
    /** how many frames to skip in disabled, to prevent overheating */
    public static final int kDisabledThrottle = 100;

    // TODO set
    public static PoseEstimateConfig mt1Config = PoseEstimateConfig.defaultMT1() // starting with the safe default means you are not required to set every single configuration value
            .withMinTagCount(1)
            .withMaxSingleTagAmbiguity(0.7) // MT1 needs low-ambiguity perspectives
            .withMaxSingleTagDistance(4.0) // If we only see one tag, don't trust it unless we are at most 4m away from it
            .withMaxAvgTagDistance(6.0) // If we see multiple tags, max avg distance must be less than 6 meters.
            .withMinAvgTagArea(0.02) // 0-100, percentage of image area. This is .02%, not 2%.
            .withFieldBounds(16.541, 8.069) // Reject pose estimates that are out of bounds.
            .withFieldBoundsMargin(0.5) // Add .5m margin around field.
            .withStdDevXY(0.5, 0.05, 2.0) // .5 base, absolute min .05, absolute max 2.0. With scaling enabled, .5 at 1m distance
            .withStdDevTheta(PoseEstimateConfig.UNTRUSTED, PoseEstimateConfig.UNTRUSTED, PoseEstimateConfig.UNTRUSTED) // never incorporate pose estimate rotation. You may want to incorporate rotation by setting these to other values
            .withStdDevDistanceScaling(1.0, 0.0, 6.0) // linear scaling, only scale between 0 and 6m.
            .withStdDevTagCountDivision(0.5); // Enhance trust by square root of number of contributing tags

    // TODO set
    public static PoseEstimateConfig mt2Config = PoseEstimateConfig.defaultMT2()
            .withMinTagCount(1)
            .withMaxSingleTagAmbiguity(1.0) // MT2 can handle maximally ambiguous perspectives. Accept tags regardless of ambiguity value.
            .withMaxSingleTagDistance(0.0) // 0 disables this check
            .withMaxAvgTagDistance(8.0)
            .withMinAvgTagArea(0.02) //0-100, percentage of image area
            .withFieldBounds(16.541, 8.069)
            .withFieldBoundsMargin(0.5)
            .withStdDevXY(0.3, 0.0001, 2.0)
            .withStdDevTheta(PoseEstimateConfig.UNTRUSTED, PoseEstimateConfig.UNTRUSTED, PoseEstimateConfig.UNTRUSTED)
            .withStdDevDistanceScaling(0.5, 0.0, 8.0) // Less aggressive STDDev scaling for MT2. Scale by sqrt(distance) rather than distance^1.
            .withStdDevTagCountDivision(0.5); // Enhance trust by a factor equal to the square root of number of contributing tags
    
    public static Pose3d kLimelightMainZeroPosition = new Pose3d(0.0392176, 0.15875, 0.5426456, Rotation3d.fromDegrees(0,-35,0)); //TODO: make sure these are right //forward is pos x, left is pos y
    
    @SuppressWarnings("resource") //Limelight objects implement AutoClosable and the compiler hates method chaining but its fine
    public static Limelight kLimelightMain = new Limelight(
      "limelight-main", //TODO: name it right
      kLimelightMainZeroPosition
    ).withPoseEstimateConfig_MT1(mt1Config)
    .withPoseEstimateConfig_MT2(mt2Config);
  }
  
  public static final class SwerveDriveConstants {
    //////////////////////////
    /// -- Drive Speeds -- ///
    //////////////////////////
    
    public static final double kDriveMaxVelocity = 5.0; // m/s
    public static final double kDrivePrecisionMultiplier = 0.5; // fractional
    
    public static final double kTurnMaxVelocity = 4.5; // rad/s
    public static final double kTurnPrecisionMultiplier = 0.5; // fractional
    
    public static final double kRobotOrientedVelocity = 2.0; // m/s
    
    ///////////////////////////
    /// -- Turn to Angle -- ///
    ///////////////////////////
    
    public static final double kTurnToAngleMaxVelocity = 6.25; // rad/s
    public static final NerdPIDConstants kTurnToAnglePIDConstants = new NerdPIDConstants(12.0, 0.0, 0.25);
    public static final Constraints kTurnToAngleTolerances = new Constraints(0.017, 0.05); 
 
    ////////////////////////////////////////////
    /// -- NerdDrivetrain Swerve Requests -- ///
    ////////////////////////////////////////////
 
    /** Used for AutoBuilder configuration */
    public static final SwerveRequest.ApplyRobotVelocity  kApplyRobotSpeedsRequest = new SwerveRequest.ApplyRobotVelocity();
    /** Robot oriented controller */
    public static final SwerveRequest.RobotCentric      kRobotOrientedSwerveRequest = 
      new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.Position);
 
    /** Field oriented controller - use @see NerdDrivertrain#resetFieldOrientation() */
    public static final SwerveRequest.FieldCentric      kFieldOrientedSwerveRequest = 
      new SwerveRequest.FieldCentric()
        .withDesaturateWheelVelocities(true)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.Position)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
 
    /** Field oriented controller - use @see NerdDrivertrain#resetFieldOrientation() */
    public static final SwerveRequest.SwerveDriveBrake  kTowSwerveRequest = 
      new SwerveRequest.SwerveDriveBrake()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.Position);
 
    ////////////////////////////////////////////
    /// -- Drive to Target Configurations -- ///
    ////////////////////////////////////////////
 
    /** @see NerdDrivetrain.driveToTarget() */
    public static final double kTargetDriveMaxLateralVelocity = 5.0;
    public static final NerdPIDConstants kTargetDriveLateralPID = new NerdPIDConstants(5.0, 0.0, 0.5);

    /** m/s and m/s/s @see NerdDrivetrain.driveToTarget() */
    public static final Constraints kTargetDriveLateralConstraints = new Constraints(kTargetDriveMaxLateralVelocity, kTargetDriveMaxLateralVelocity);
    public static final double kTargetDriveMaxRotationalVelocity = 9.4;
    public static final NerdPIDConstants kTargetDriveRotationalPID = new NerdPIDConstants(4.0, 0.0, 0.2);
 
    /** rad/s and rad/s/s @see NerdDrivetrain.driveToTarget() */
    public static final Constraints kTargetDriveRotationalConstraints = new Constraints(kTargetDriveMaxRotationalVelocity, kTargetDriveMaxRotationalVelocity);
 
    public static final MultiProfiledPIDController kTargetDriveController = new MultiProfiledPIDController()
      .add("x", kTargetDriveLateralPID, kTargetDriveLateralConstraints, 0.1, 0.1)
      .add("y", kTargetDriveLateralPID, kTargetDriveLateralConstraints, 0.1, 0.1)
      .add("r", kTargetDriveRotationalPID, kTargetDriveRotationalConstraints, 0.05, 0.2)
      .withContinuousInput("r", -Math.PI, Math.PI);
 
    public static enum FieldPositions {
      // Add field positions
      HUB_CENTER(4.626, 4.035, 0.0),
      DEPOT_PASSING(3.25,6.0,0.0),
      OUTPOST_PASSING(3.25,2.0,0.0);

      public Pose2d blue, red; // meters and degrees
      FieldPositions(double _blueX, double _blueY, double _blueHeadingDegrees) {
        blue = new Pose2d(new Translation2d(_blueX, _blueY), new Rotation2d(Units.degreesToRadians(_blueHeadingDegrees)));
        red = NerdFlippingUtil.flipFieldPose(blue);
      }

      public Pose2d get() {
        RobotContainer.refreshAlliance();
        return RobotContainer.IsRedSide() ? this.red : this.blue;
      }
    }
  }

  public static final class ZoneConstants {
    public static final double kHubRadius = 4.5;
    public static final double kTrenchWidth = 1.2;
    public static final double kDistFromCenterLongPass = 0.0;

    private static final NerdZone kBlueTrench = 
      new RectangleZone(
        new Pose2d(FieldPositions.HUB_CENTER.blue.getX() - (kTrenchWidth/2.0), -10, Rotation2d.ZERO), 
        new Pose2d(FieldPositions.HUB_CENTER.blue.getX() + (kTrenchWidth/2.0), 20, Rotation2d.ZERO));
    private static final NerdZone kRedTrench = 
      new RectangleZone(
        new Pose2d(FieldPositions.HUB_CENTER.red.getX() - (kTrenchWidth/2.0), -10, Rotation2d.ZERO), 
        new Pose2d(FieldPositions.HUB_CENTER.red.getX() + (kTrenchWidth/2.0), 20, Rotation2d.ZERO));

    // private static final NerdZone kBlueHub = new SemicircleZone(FieldPositions.HUB_CENTER.blue, ZoneConstants.kHubRadius);
    // private static final NerdZone kRedHub = new SemicircleZone(FieldPositions.HUB_CENTER.red, ZoneConstants.kHubRadius);

    public static final NerdZone kNoShootZone = new ZoneGroup(
      kBlueTrench,
      kRedTrench
    );
  }
  
  // ------------------------ subsystem constants ------------------------------- //
  public static final class TurretSwivelConstants {
    public static final int kMotor1ID = 45;
    public static final int kCANdiID = 50;

    private static final Slot0Configs kSlot0Configs = 
      new Slot0Configs()
        .withKP(0.5)
        .withKI(0)
        .withKD(0);

    public static final MotorOutputConfigs kMotorOutputConfigs = 
      new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive); // placeholder, but based on CAD, CW should move the hood up
      
    public static final CurrentLimitsConfigs kMotorCurrentLimitsConfigs = 
      new CurrentLimitsConfigs()
        .withStatorCurrentLimit(30)
        .withStatorCurrentLimitEnable(true);

    private static final MotionMagicConfigs kMotionMagicConfigs = 
      new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(10)
        .withMotionMagicAcceleration(25);
    
    private static final FeedbackConfigs kFeedbackConfigs = 
      new FeedbackConfigs()
        .withFeedbackRemoteSensorID(kCANdiID)
        .withFeedbackRotorOffset(0.0)
        .withSensorToMechanismRatio(200.0/18.0)
      ;

    public static final TalonFXConfiguration kSubsystemConfiguration = 
      new TalonFXConfiguration()
        .withSlot0(kSlot0Configs)
        .withCurrentLimits(kMotorCurrentLimitsConfigs)
        .withMotionMagic(kMotionMagicConfigs)
        .withMotorOutput(kMotorOutputConfigs)
        .withFeedback(kFeedbackConfigs);

    public static final Pose2d kTurretSwivelRelativePosition = new Pose2d(-0.1234186,0.15875, Rotation2d.ZERO); //TODO: SET VALUES // forward is pos x, left is pos y
    public static final TurretSwivelConfiguration kTurretSwivelConfiguration = new TurretSwivelConfiguration()
      .setLowerAngleBound(0)
      .setUpperAngleBound(313)
      .setMaxAngleTolerance(2)
      .setMaxVelocityTolerance(3)
      .setRelativePosition(kTurretSwivelRelativePosition); 
  }

  public static final class IntakeRollerConstants {
    public static final int kMotor1ID = 15;
    public static final int kMotor2ID = 8;

    private static final Slot0Configs kSlot0Configs = 
      new Slot0Configs()
        .withKP(0.5)
        .withKI(0.0)
        .withKD(0.0);

    public static final MotorOutputConfigs kMotorOutputConfigs = 
      new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    private static final CurrentLimitsConfigs kCurrentLimitsConfigs = 
      new CurrentLimitsConfigs()
        .withStatorCurrentLimit(80)
        .withStatorCurrentLimitEnable(true);

    private static final FeedbackConfigs kFeedbackConfigs = 
      new FeedbackConfigs()
        .withRotorToSensorRatio(1.0)
      ;

    public static final TalonFXConfiguration kSubsystemConfiguration = 
      new TalonFXConfiguration()
        .withSlot0(kSlot0Configs)
        .withCurrentLimits(kCurrentLimitsConfigs)
        .withFeedback(kFeedbackConfigs);
  }

  public static final class ThroatConstants {
    public static final int kMotor1ID = 27;

    public static final Slot0Configs kSlot0Configs = 
      new Slot0Configs()
        .withKP(0.5)
        .withKI(0.0)
        .withKD(0.0);

    public static final MotorOutputConfigs kMotorOutputConfigs =
      new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    public static final CurrentLimitsConfigs kCurrentLimitsConfigs = 
      new CurrentLimitsConfigs()
        .withStatorCurrentLimit(50)
        .withStatorCurrentLimitEnable(true);
        
    public static final TalonFXConfiguration kSubsystemConfiguration = 
      new TalonFXConfiguration()
        .withSlot0(kSlot0Configs)
        .withMotorOutput(kMotorOutputConfigs)
        .withCurrentLimits(kCurrentLimitsConfigs);
  }

  public static final class ConveyorBeltConstants {
    public static final int kMotor1ID = 26;

    public static final Slot0Configs kSlot0Configs = 
      new Slot0Configs()
      //TODO
        .withKP(0.5)
        .withKI(0.0)
        .withKD(0.0);
        
    public static final MotorOutputConfigs kMotorOutputConfigs = 
      new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
        

    public static final CurrentLimitsConfigs kCurrentLimitsConfigs =
      new CurrentLimitsConfigs()
      //TODO
        .withStatorCurrentLimit(50)
        .withStatorCurrentLimitEnable(true);

    public static final TalonFXConfiguration kSubsystemConfiguration =
      new TalonFXConfiguration()
        .withSlot0(kSlot0Configs)
        .withMotorOutput(kMotorOutputConfigs)
        .withCurrentLimits(kCurrentLimitsConfigs);
  }

    public static final class RollerFloor {
    public static final int kMotor1ID = 0;

    public static final Slot0Configs kSlot0Configs = 
      new Slot0Configs()
      //TODO
        .withKP(0.5)
        .withKI(0.0)
        .withKD(0.0);
        
    public static final MotorOutputConfigs kMotorOutputConfigs = 
      new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
        

    public static final CurrentLimitsConfigs kCurrentLimitsConfigs =
      new CurrentLimitsConfigs()
      //TODO
        .withStatorCurrentLimit(50)
        .withStatorCurrentLimitEnable(true);

    public static final TalonFXConfiguration kSubsystemConfiguration =
      new TalonFXConfiguration()
        .withSlot0(kSlot0Configs)
        .withMotorOutput(kMotorOutputConfigs)
        .withCurrentLimits(kCurrentLimitsConfigs);
  }

public static final class ShooterConstants {
    public static final int kMotor1ID = 35;
    public static final int kMotor2ID = 36;

    public static final Slot0Configs kSlot0Configs = 
      new Slot0Configs()
      //TODO
        .withKP(0.5)
        .withKI(0.0)
        .withKD(0.0);
        
    public static final MotorOutputConfigs kMotorOutputConfigs = 
      new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
        
    public static final CurrentLimitsConfigs kCurrentLimitsConfigs =
      new CurrentLimitsConfigs()
      //TODO
        .withStatorCurrentLimit(50)
        .withStatorCurrentLimitEnable(true);

    public static final TalonFXConfiguration kSubsystemConfiguration =
      new TalonFXConfiguration()
        .withSlot0(kSlot0Configs)
        .withMotorOutput(kMotorOutputConfigs)
        .withCurrentLimits(kCurrentLimitsConfigs);
    
    public static final double kLookAheadFactor = 0.0; //TODO test this

    public static final LInTable kShooterTable = new LInTable(
      new double[] {},
      new double[] {}
    );

    public static final double kVelocityTolerance = 0.5; //TODO test this
  }

  public static final class HoodConstants {
    public static final int kMotor1ID = 37;
    public static final int kCANcoderID = 40; // TODO change to the actual ID

    public static final Slot0Configs kSlot0Configs = 
      new Slot0Configs()
      //TODO
        .withKP(0.5)
        .withKI(0.0)
        .withKD(0.0);
        
    public static final MotorOutputConfigs kMotorOutputConfigs = 
      new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
        
    public static final CurrentLimitsConfigs kCurrentLimitsConfigs =
      new CurrentLimitsConfigs()
      //TODO
        .withStatorCurrentLimit(50)
        .withStatorCurrentLimitEnable(true);
    private static final FeedbackConfigs kFeedbackConfigs =
      new FeedbackConfigs()
          .withFeedbackRemoteSensorID(kCANcoderID)
          .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
          .withFeedbackRotorOffset(0.0)
          .withSensorToMechanismRatio(1.0)
        ;
    public static final TalonFXConfiguration kSubsystemConfiguration =
      new TalonFXConfiguration()
        .withSlot0(kSlot0Configs)
        .withMotorOutput(kMotorOutputConfigs)
        .withCurrentLimits(kCurrentLimitsConfigs)
        .withFeedback(kFeedbackConfigs);
    
    public static final double kUpPos = 1.0; 
    public static final double kDownPos = 0.0;

    public static final LInTable kHoodTable = new LInTable(
      new double[] {},
      new double[] {}
    );
  }

  public static final class IntakeSlideConstants {
    public static final int kMotor1ID = 17;

    public static final Slot0Configs kSlot0Configs = 
      new Slot0Configs()
      //TODO
      .withKP(0.5)
      .withKI(0.0)
      .withKD(0.0);

    public static final MotorOutputConfigs kMotorOutputConfigs =
      new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    
    public static final CurrentLimitsConfigs kCurrentLmitsConfigs = 
      new CurrentLimitsConfigs()
      //TODO
        .withStatorCurrentLimit(50)
        .withStatorCurrentLimitEnable(true);

    public static final TalonFXConfiguration kSubsystemConfiguraion = 
      new TalonFXConfiguration()
        .withSlot0(kSlot0Configs)
        .withMotorOutput(kMotorOutputConfigs)
        .withCurrentLimits(kCurrentLmitsConfigs);
  }

// Template for subsystem based constants
// Replicate for every subsystem
//e.g. IndexerConstants
  // public static final class MechanismConstants {
  //   public static final int kMotor1ID = 0;
  //   //public static final int kMotor2ID = 0;


  //   private static final Slot0Configs kSlot0Configs = 
  //     new Slot0Configs()
  //       .withKP(0.0)
  //       .withKI(0.0)
  //       .withKD(0.0);
  //       // .withKV(0.0)
  //       // .withKS(0.0)


  //   private static final MotorOutputConfigs kMotorOutputConfigs =
  //     new MotorOutputConfigs()
  //       .withInverted(InvertedValue.CounterClockwise_Positive)
  //       .withNeutralMode(NeutralModeValue.Brake);

  //   private static final CurrentLimitsConfigs kCurrentLimitsConfigs =
  //     new CurrentLimitsConfigs()
  //       .withStatorCurrentLimit(40)
  //       .withStatorCurrentLimitEnable(true);

  //   public static final TalonFXConfiguration kSubsystemConfiguration = 
  //     new TalonFXConfiguration()
  //       .withSlot0(kSlot0Configs)
  //       .withMotorOutput(kMotorOutputConfigs)
  //       // .withMotionMagic(kMotionMagicConfigs)
  //       .withCurrentLimits(kCurrentLimitsConfigs);
  // }

  /**
 * Stores all robot subsystem instances.
 *
 * Add new subsystems here as they are created.
 */
  public static final class Subsystems {

      // =========================
      // Example Subsystem Template
      // =========================
      //
      // Uncomment and modify when creating a new mechanism.
      //
      // public static final boolean useExample = true;
      //
      // public static final TemplateSubsystem example =
      //     (!USE_SUBSYSTEMS) ? null :
      //     new TemplateSubsystem(
      //         "Example",
      //         ExampleConstants.kMotorID,
      //         SubsystemMode.VOLTAGE,
      //         0.0,
      //         useExample)
      //     .configureMotors(ExampleConstants.kSubsystemConfiguration);

      public static final boolean useTurretSwivel = true;
      public static final TurretSwivel turretSwivel = (!USE_SUBSYSTEMS) ? null :
        (TurretSwivel) new TurretSwivel(
            "Turret",
            TurretSwivelConstants.kMotor1ID,
            SubsystemMode.POSITION,
            0.0,
            useTurretSwivel)
          .setTurretSwivelConfiguration(TurretSwivelConstants.kTurretSwivelConfiguration)
          .configureMotors(TurretSwivelConstants.kSubsystemConfiguration);

      public static final boolean useShooter = true;
      public static final TemplateSubsystem shooter = (!USE_SUBSYSTEMS) ? null :
        new TemplateSubsystem(
          "Shooter", 
          ShooterConstants.kMotor1ID, 
          SubsystemMode.VELOCITY, 
          0.0, 
          useShooter)
        .addMotor(ShooterConstants.kMotor2ID, MotorAlignmentValue.Opposed)
        .configureMotors(ShooterConstants.kSubsystemConfiguration);

      public static final boolean useIntakeRoller = true;
      public static final TemplateSubsystem intakeRoller = (!USE_SUBSYSTEMS) ? null :
        new TemplateSubsystem(
            "Intake Roller", 
            IntakeRollerConstants.kMotor1ID, 
            SubsystemMode.VOLTAGE, 
            0.0,
            useIntakeRoller)
        .configureMotors(IntakeRollerConstants.kSubsystemConfiguration);

      public static final boolean useIntakeSlide = true;
      public static final TemplateSubsystem intakeSlide = (!USE_SUBSYSTEMS) ? null:
        new TemplateSubsystem(
          "Intake Slide", 
          IntakeSlideConstants.kMotor1ID, 
          SubsystemMode.VOLTAGE, 
          0.0, 
          useIntakeRoller)
        .configureMotors(IntakeSlideConstants.kSubsystemConfiguraion);

      public static final boolean useThroat = true;
      public static final TemplateSubsystem throat = (!USE_SUBSYSTEMS) ? null :
        new TemplateSubsystem(
            "Throat", 
            ThroatConstants.kMotor1ID, 
            SubsystemMode.VOLTAGE, 
            0.0,
            useThroat)
          .configureMotors(ThroatConstants.kSubsystemConfiguration);

      public static final boolean useConveyorBelt = true;
      public static final TemplateSubsystem conveyorBelt = (!USE_SUBSYSTEMS) ? null :
        new TemplateSubsystem(
          "Conveyor Belt", 
          ConveyorBeltConstants.kMotor1ID, 
          SubsystemMode.VELOCITY, 
          0.0, 
        useConveyorBelt)
        .configureMotors(ConveyorBeltConstants.kSubsystemConfiguration);

      public static final boolean useRollerFloor = true;
      public static final TemplateSubsystem rollerFloor = (!USE_SUBSYSTEMS) ? null :
        new TemplateSubsystem(
          "Roller Floor", 
          RollerFloor.kMotor1ID, 
          SubsystemMode.VELOCITY, 
          0.0, 
        useConveyorBelt)
        .configureMotors(RollerFloor.kSubsystemConfiguration);

      public static final boolean useHood = true;
      public static final TemplateSubsystem hood = (!USE_SUBSYSTEMS) ? null :
        new TemplateSubsystem(
          "Hood", 
          HoodConstants.kMotor1ID, 
          SubsystemMode.POSITION,
          0.0, 
          useHood)
        .configureMotors(HoodConstants.kSubsystemConfiguration);

      public static void init() {}
  }
}
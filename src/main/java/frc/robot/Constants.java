// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot;
 
import java.util.function.BiFunction;
import java.util.function.Function;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.util.FlippingUtil;
 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.subsystems.template.TemplateSubsystem.SubsystemMode;
import frc.robot.util.MultiProfiledPIDController;
import frc.robot.util.NerdyMath;
import frc.robot.util.Translation2dSlewRateLimiter;
import frc.robot.util.logging.Reportable.LOG_LEVEL;
 
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
  public static final LOG_LEVEL ROBOT_LOG_LEVEL = LOG_LEVEL.MEDIUM;
  
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
          return Translation2d.kZero;
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
  
  public static final class SwerveDriveConstants {
    //////////////////////////
    /// -- Drive Speeds -- ///
    //////////////////////////
    
    public static final double kDriveMaxVelocity = 0.0; // m/s TODO: tune
    public static final double kDrivePrecisionMultiplier = 0.5; // fractional
 
    public static final double kTurnMaxVelocity = 0.0; // rad/s TODO: tune
    public static final double kTurnPrecisionMultiplier = 0.5; // fractional
 
    public static final double kRobotOrientedVelocity = 0.0; // m/s TODO: tune
 
    ///////////////////////////
    /// -- Turn to Angle -- ///
    ///////////////////////////
 
    public static final double kTurnToAngleMaxVelocity = 0.0; // rad/s TODO: tune
    public static final PIDConstants kTurnToAnglePIDConstants = new PIDConstants(0.0, 0.0, 0.0); // TODO: tune
    public static final Constraints kTurnToAngleTolerances = new Constraints(0.0, 0.0); // TODO: tune
 
    ////////////////////////////////////////////
    /// -- NerdDrivetrain Swerve Requests -- ///
    ////////////////////////////////////////////
 
    /** Used for AutoBuilder configuration */
    public static final SwerveRequest.ApplyRobotSpeeds  kApplyRobotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();
    /** Robot oriented controller */
    public static final SwerveRequest.RobotCentric      kRobotOrientedSwerveRequest = 
      new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.Position);
 
    /** Field oriented controller - use @see NerdDrivertrain#resetFieldOrientation() */
    public static final SwerveRequest.FieldCentric      kFieldOrientedSwerveRequest = 
      new SwerveRequest.FieldCentric()
        .withDesaturateWheelSpeeds(true)
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
    public static final double kTargetDriveMaxLateralVelocity = 0.0; // TODO: tune
    public static final PIDConstants kTargetDriveLateralPID = new PIDConstants(0.0, 0.0, 0.0); // TODO: tune
 
    /** m/s and m/s/s @see NerdDrivetrain.driveToTarget() */
    public static final Constraints kTargetDriveLateralConstraints = new Constraints(kTargetDriveMaxLateralVelocity, kTargetDriveMaxLateralVelocity);
    public static final double kTargetDriveMaxRotationalVelocity = 0.0; // TODO: tune
    public static final PIDConstants kTargetDriveRotationalPID = new PIDConstants(0.0, 0.0, 0.0); // TODO: tune
 
    /** rad/s and rad/s/s @see NerdDrivetrain.driveToTarget() */
    public static final Constraints kTargetDriveRotationalConstraints = new Constraints(kTargetDriveMaxRotationalVelocity, kTargetDriveMaxRotationalVelocity);
 
    public static final MultiProfiledPIDController kTargetDriveController = new MultiProfiledPIDController()
      .add("x", kTargetDriveLateralPID, kTargetDriveLateralConstraints, 0.1, 0.1)
      .add("y", kTargetDriveLateralPID, kTargetDriveLateralConstraints, 0.1, 0.1)
      .add("r", kTargetDriveRotationalPID, kTargetDriveRotationalConstraints, 0.05, 0.2)
      .withContinuousInput("r", -Math.PI, Math.PI);
 
    public static enum FieldPositions {
      // Add field positions
      HUB_CENTER(4.626, 4.035, 0.0);
      
      public Pose2d blue, red; // meters and degrees
      FieldPositions(double _blueX, double _blueY, double _blueHeadingDegrees) {
        blue = new Pose2d(new Translation2d(_blueX, _blueY), new Rotation2d(Units.degreesToRadians(_blueHeadingDegrees)));
        red = FlippingUtil.flipFieldPose(blue);
      }

      public Pose2d get() {
        RobotContainer.refreshAlliance();
        return RobotContainer.IsRedSide() ? this.red : this.blue;
      }
    }
  }

  public static final class RingDriveConstants {
    public static final double kInitialDistance = 0.2; // m
    public static final double kDriveVelocity = 1.0; // m/s
    public static final double kMaximumDistance = 1.0; // m
    public static final double kMinimumDistance = 0.2; // m
    public static final double kRobotRotationOffset = Math.PI; // rad
  }

  public static final class PathPlannerConstants {
    public static final double kPP_P = 0.0; // TODO: tune
    public static final double kPP_I = 0.0;
    public static final double kPP_D = 0.0;
 
    public static final PIDConstants kPPTranslationPIDConstants = new PIDConstants(kPP_P, kPP_I, kPP_D);

    public static final double kPP_ThetaP = 0.0; // TODO: tune
    public static final double kPP_ThetaI = 0;
    public static final double kPP_ThetaD = 0.0; // TODO: tune
 
    public static final PIDConstants kPPRotationPIDConstants = new PIDConstants(kPP_ThetaP, kPP_ThetaI, kPP_ThetaD);
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

    public static enum Camera {
      // TODO: Update the limelights
      Front("limelight-fr", "10.6.87.17:5802"),
      Back("limelight-br", "10.6.87.15:5802");
      ;

      public final String name, ip;
      Camera(String name, String ip) {
        this.name = name;
        this.ip = ip;
      }
    }
  }

// Template for subsystem based constants
// Replicate for every subsystem
//e.g. IndexerConstants
  public static final class MechanismConstants {

  
    public static final int kMotor1ID = 0;
    //public static final int kMotor2ID = 0;


    private static final Slot0Configs kSlot0Configs = 
      new Slot0Configs()
        .withKP(0.0)
        .withKI(0.0)
        .withKD(0.0);
        // .withKV(0.0)
        // .withKS(0.0)


    private static final MotorOutputConfigs kMotorOutputConfigs =
      new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    private static final CurrentLimitsConfigs kCurrentLimitsConfigs =
      new CurrentLimitsConfigs()
        .withStatorCurrentLimit(40)
        .withStatorCurrentLimitEnable(true);

    public static final TalonFXConfiguration kSubsystemConfiguration = 
      new TalonFXConfiguration()
        .withSlot0(kSlot0Configs)
        .withMotorOutput(kMotorOutputConfigs)
        // .withMotionMagic(kMotionMagicConfigs)
        .withCurrentLimits(kCurrentLimitsConfigs);
  }
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

      public static void init() {}
  }
}
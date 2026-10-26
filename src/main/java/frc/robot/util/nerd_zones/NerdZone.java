package frc.robot.util.nerd_zones;

import org.wpilib.math.geometry.Pose2d;

public interface NerdZone {
    boolean check(Pose2d robotPose);
}



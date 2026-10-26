package frc.robot.util.nerd_zones;

import org.wpilib.math.geometry.Pose2d;

public class SemicircleZone extends CircleZone {
    public SemicircleZone(Pose2d center, double radius) {
        super(center, radius);
    }

    @Override
    public boolean check(Pose2d robotPose) {
        if (!super.check(robotPose)) return false;
        //dot
        return ((robotPose.getX() - center.getX()) * center.getRotation().getCos() + (robotPose.getY() - center.getY()) * center.getRotation().getSin()) > 0;
    }
}

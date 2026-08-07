package frc.robot.vision;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public class VisionSys {
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    Map<Pose2d, Integer> reefPoses = new HashMap<>();
    
    public VisionSys() {
        layout.getTags().stream().forEach(tag -> {
            if (tag.ID >= 1 && tag.ID <= 16)
                reefPoses.put(tag.pose.toPose2d(), tag.ID);
            else if (tag.ID >= 17 && tag.ID <= 32)
                reefPoses.put(tag.pose.toPose2d(), tag.ID);
        });
    }
}

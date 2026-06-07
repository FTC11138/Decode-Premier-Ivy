package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.Constants;

public class TurretLocation {
    public static Pose getTurretPose(Pose robotPose) {
        double heading = robotPose.getHeading();
        return new Pose(
                robotPose.getX() + Constants.turretForwardOffsetInches * Math.cos(heading) - Constants.turretLeftOffsetInches * Math.sin(heading),
                robotPose.getY() + Constants.turretForwardOffsetInches * Math.sin(heading) + Constants.turretLeftOffsetInches * Math.cos(heading),
                heading
        );
    }
}

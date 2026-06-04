package org.firstinspires.ftc.teamcode.math;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class TurretLocation {
    public static double turretForwardOffsetInches = -1.496;
    public static double turretLeftOffsetInches = 0;

    public static Pose getTurretPose(Pose robotPose) {
        double heading = robotPose.getHeading();
        return new Pose(
                robotPose.getX() + turretForwardOffsetInches * Math.cos(heading) - turretLeftOffsetInches * Math.sin(heading),
                robotPose.getY() + turretForwardOffsetInches * Math.sin(heading) + turretLeftOffsetInches * Math.cos(heading),
                heading
        );
    }
}

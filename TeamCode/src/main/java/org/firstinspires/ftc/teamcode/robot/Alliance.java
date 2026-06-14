package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.Constants;

public enum Alliance {
    RED,
    BLUE;

    public static Alliance current = Alliance.BLUE;

    public Pose getGoal() {
        if (this == RED) {
            return new Pose(Constants.redGoalX, Constants.redGoalY);
        }
        return new Pose(Constants.blueGoalX, Constants.blueGoalY);
    }
}

package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;

import static org.firstinspires.ftc.teamcode.math.PoseMirror.mirror;

public enum Alliance {
    RED(new Pose(138, 138)),
    BLUE(mirror(new Pose(138, 138)));

    public static Alliance current = Alliance.RED;
    public final Pose goal;

    Alliance(Pose goal) {
        this.goal = goal;
    }
}
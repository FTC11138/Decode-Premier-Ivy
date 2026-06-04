package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

public class PoseMirror {
    public static Pose mirror(Pose pose) {
        return new Pose(141.5 - pose.getX(), pose.getY(), Math.PI - pose.getHeading());
    }
}
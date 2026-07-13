package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.Constants;

public class PoseMirror {
    public static Pose mirror(Pose pose) {
        return new Pose(Constants.fieldWidthInches - pose.getX(), pose.getY(), Math.PI - pose.getHeading());
    }
}
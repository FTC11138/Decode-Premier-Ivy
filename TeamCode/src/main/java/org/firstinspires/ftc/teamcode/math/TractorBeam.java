package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;

public class TractorBeam {
    public static void aimTurret(Pose currentPose, Robot robot, Alliance alliance) {
        double turretTargetDegrees = getTurretTargetDegrees(currentPose, robot.telemetry, alliance);

        robot.turret.setTargetDegrees(turretTargetDegrees);
    }

    public static double getTurretTargetDegrees(Pose currentPose, Telemetry telemetry, Alliance alliance) {
        Pose turretPose = TurretLocation.getTurretPose(currentPose);
        Pose goal = alliance.goal;

        double fieldTargetRadians = Math.atan2(
                goal.getY() - turretPose.getY(),
                goal.getX() - turretPose.getX()
        );
        double fieldTargetDegrees = Math.toDegrees(fieldTargetRadians);
        double turretTargetDegrees = AngleUnit.normalizeDegrees(fieldTargetDegrees - Math.toDegrees(turretPose.getHeading()));

        if (turretTargetDegrees < Constants.turretMinimumAutoAimAngleDegrees) {
            turretTargetDegrees += 360;
        }

        telemetry.addData("Auto Aim Field Target", fieldTargetDegrees);
        telemetry.addData("Auto Aim Turret Target", turretTargetDegrees);

        return turretTargetDegrees;
    }
}

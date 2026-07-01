package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import com.qualcomm.robotcore.util.Range;

public class TractorBeam {
    public static void aimTurret(Pose currentPose, Robot robot, Alliance alliance) {
        double turretTargetDegrees = getTurretTargetDegrees(currentPose, robot.telemetry, alliance);

        robot.turret.setTargetDegrees(turretTargetDegrees);
    }

    public static double getTurretTargetDegrees(Pose currentPose, Telemetry telemetry, Alliance alliance) {
        Pose turretOnlyCorrectedRobotPose = new Pose(
                currentPose.getX() + Constants.turretPoseXCorrectionInches,
                currentPose.getY(),
                currentPose.getHeading()
        );
        Pose turretPose = TurretLocation.getTurretPose(turretOnlyCorrectedRobotPose);
        Pose goal = alliance.getGoal();

        double fieldTargetRadians = Math.atan2(
                goal.getY() - turretPose.getY(),
                goal.getX() - turretPose.getX()
        );
        double fieldTargetDegrees = Math.toDegrees(fieldTargetRadians);
        double turretTargetDegrees = AngleUnit.normalizeDegrees(
                fieldTargetDegrees
                        - Math.toDegrees(turretPose.getHeading())
                        + Constants.turretAimOffsetDegrees
        );

        turretTargetDegrees = Range.clip(
                turretTargetDegrees,
                Constants.turretMinAngleDegrees,
                Constants.turretMaxAngleDegrees
        );

        telemetry.addData("Auto Aim Field Target", fieldTargetDegrees);
        telemetry.addData("Auto Aim Turret Target", turretTargetDegrees);
        telemetry.addData("Auto Aim Offset", Constants.turretAimOffsetDegrees);
        telemetry.addData("Auto Aim X Correction", Constants.turretPoseXCorrectionInches);

        return turretTargetDegrees;
    }
}

package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "LL Pose Resetter", group = "Utility")
public class LLPoseResetter extends OpMode {
    private Limelight3A camera;
    private Follower follower;
    private boolean crossWasDown = false;
    private String resetStatus = "Press gamepad1 cross to reset pose";

    @Override
    public void init() {
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
    }

    @Override
    public void start() {
        camera.start();
    }

    @Override
    public void loop() {
        follower.update();

        Pose pose = follower.getPose();
        telemetry.addData("Pose X", pose.getX());
        telemetry.addData("Pose Y", pose.getY());
        telemetry.addData("Pose Heading", Math.toDegrees(pose.getHeading()));
        telemetry.addData("LL Reset", resetStatus);
        telemetry.update();
    }

    @Override
    public void stop() {
        camera.stop();
    }

    private void resetPoseFromLimelight() {
        LLResult result = camera.getLatestResult();
        if (result == null) {
            resetStatus = "No Limelight result";
            return;
        }

        if (!result.isValid()) {
            resetStatus = "Limelight result invalid";
            return;
        }

        if (result.getBotposeTagCount() <= 0) {
            resetStatus = "No AprilTags in botpose";
            return;
        }

        Pose cameraPose = getRobotPoseFromCamera(result.getBotpose());
        follower.setPose(cameraPose);
        resetStatus = String.format(
                "Reset to x=%.1f y=%.1f h=%.1f",
                cameraPose.getX(),
                cameraPose.getY(),
                Math.toDegrees(cameraPose.getHeading())
        );
    }

    private Pose getRobotPoseFromCamera(Pose3D botpose) {
        Position position = botpose.getPosition().toUnit(DistanceUnit.INCH);
        double headingRadians = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

        return new Pose(position.x, position.y, headingRadians, FTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}

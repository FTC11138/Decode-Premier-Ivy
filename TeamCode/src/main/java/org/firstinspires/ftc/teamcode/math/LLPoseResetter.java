package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.HardwareNames;

public class LLPoseResetter {
    private final Limelight3A camera;
    private String status = "Waiting for reset";

    public LLPoseResetter(HardwareMap hardwareMap) {
        camera = hardwareMap.get(Limelight3A.class, HardwareNames.limelight);
    }

    public void start() {
        camera.start();
    }

    public void stop() {
        camera.stop();
    }

    /**
     * Feed the robot's current field yaw to the camera so MegaTag2 can resolve a
     * stable pose. Call this every loop (not just at reset time) so the yaw the
     * Limelight uses is always fresh - MT2 is only as good as the heading it's given.
     * The heading is converted from Pedro's frame to the FTC field frame the
     * Limelight reports in.
     */
    public void updateOrientation(Drivetrain drivetrain) {
        Pose pedroPose = drivetrain.getPose();
        if (pedroPose == null) return;

        Pose ftcPose = new Pose(
                pedroPose.getX(),
                pedroPose.getY(),
                pedroPose.getHeading(),
                PedroCoordinates.INSTANCE
        ).getAsCoordinateSystem(FTCCoordinates.INSTANCE);

        camera.updateRobotOrientation(Math.toDegrees(ftcPose.getHeading()));
    }

    public boolean resetPose(Drivetrain drivetrain) {
        // Make sure MegaTag2 has the current yaw before we read its pose. For the
        // freshest result, callers should also call updateOrientation() every loop.
        updateOrientation(drivetrain);

        Pose cameraPose = getRobotPoseFromCamera();
        if (cameraPose == null) return false;

        if (!isReasonableReset(drivetrain.getPose(), cameraPose)) {
            return false;
        }

        drivetrain.setPose(cameraPose);
        status = String.format(
                "Reset to x=%.1f y=%.1f h=%.1f",
                cameraPose.getX(),
                cameraPose.getY(),
                Math.toDegrees(cameraPose.getHeading())
        );
        return true;
    }

    public String getStatus() {
        return status;
    }

    private Pose getRobotPoseFromCamera() {
        LLResult result = camera.getLatestResult();
        if (result == null) {
            status = "No Limelight result";
            return null;
        }

        if (!result.isValid()) {
            status = "Limelight result invalid";
            return null;
        }

        int tagCount = result.getBotposeTagCount();
        if (tagCount < Constants.limelightMinimumTagCount) {
            status = String.format(
                    "Need %d tags, saw %d",
                    Constants.limelightMinimumTagCount,
                    tagCount
            );
            return null;
        }

        return convertToPedroPose(result.getBotpose_MT2());
    }

    private boolean isReasonableReset(Pose currentPose, Pose cameraPose) {
        if (Constants.limelightAllowLargePoseReset) {
            return true;
        }

        double distanceError = Math.hypot(
                cameraPose.getX() - currentPose.getX(),
                cameraPose.getY() - currentPose.getY()
        );
        double headingErrorDegrees = Math.abs(AngleUnit.normalizeDegrees(
                Math.toDegrees(cameraPose.getHeading() - currentPose.getHeading())
        ));

        if (distanceError > Constants.limelightMaxResetDistanceInches) {
            status = String.format(
                    "Rejected LL reset: %.1f in jump > %.1f",
                    distanceError,
                    Constants.limelightMaxResetDistanceInches
            );
            return false;
        }

        if (headingErrorDegrees > Constants.limelightMaxResetHeadingDegrees) {
            status = String.format(
                    "Rejected LL reset: %.1f deg jump > %.1f",
                    headingErrorDegrees,
                    Constants.limelightMaxResetHeadingDegrees
            );
            return false;
        }

        return true;
    }

    private Pose convertToPedroPose(Pose3D botpose) {
        Position position = botpose.getPosition().toUnit(DistanceUnit.INCH);
        double headingRadians = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

        return new Pose(position.x, position.y, headingRadians, FTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}

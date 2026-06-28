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
    // Soft-fusion parameters — tuned conservatively for competition reliability.
    // ALPHA=0.15: each correction frame moves pose 15% toward camera; a single
    // bad frame can only nudge pose by 15% of its error, not slam it to wrong value.
    // Raise toward 0.25 if drift correction feels too slow in practice.
    private static final double FUSION_ALPHA = 0.15;
    // Require this many consecutive valid frames before any correction fires.
    // At ~15 Hz camera update rate, 3 frames = ~200ms consensus window.
    private static final int FUSION_MIN_FRAMES = 3;
    // Robot must be moving slower than this for camera readings to be usable.
    // At 15 in/s and ~150ms camera latency, positional error from motion is ~2.25 in.
    private static final double FUSION_MAX_VELOCITY_IPS = 15.0;
    // Reject camera pose if it puts the robot more than this far from odometry.
    // Catches the PnP "wrong solution" which typically places the robot 36-60 in off.
    private static final double FUSION_MAX_POSITION_JUMP_IN = 30.0;
    // Reject camera pose if its heading disagrees with odometry by more than this.
    // The PnP flipped solution usually differs by ~60 degrees; this gate catches it.
    private static final double FUSION_MAX_HEADING_DIFF_DEG = 25.0;

    private final Limelight3A camera;
    private String status = "Waiting";
    private int consecutiveGoodFrames = 0;

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
     * Called every loop iteration. Applies a gentle pose correction whenever
     * all quality gates pass for FUSION_MIN_FRAMES consecutive frames.
     * Safe to call unconditionally — does nothing when camera is unavailable,
     * robot is moving fast, or the camera reading looks suspect.
     */
    public void periodicUpdate(Drivetrain drivetrain) {
        Pose cameraPose = getRobotPoseFromCamera();
        if (cameraPose == null) {
            consecutiveGoodFrames = 0;
            return;
        }

        Pose odoPose = drivetrain.getPose();

        // Gate 1: robot must be moving slowly enough for camera to be accurate.
        if (drivetrain.getVelocityInchesPerSecond() > FUSION_MAX_VELOCITY_IPS) {
            consecutiveGoodFrames = 0;
            status = String.format("Moving %.1f in/s — fusion paused", drivetrain.getVelocityInchesPerSecond());
            return;
        }

        // Gate 2: camera position must be plausible relative to odometry.
        // The PnP wrong solution often places the robot on the opposite side of the field.
        double positionDelta = Math.hypot(
                cameraPose.getX() - odoPose.getX(),
                cameraPose.getY() - odoPose.getY()
        );
        if (positionDelta > FUSION_MAX_POSITION_JUMP_IN) {
            consecutiveGoodFrames = 0;
            status = String.format("Position gap %.1f in — rejected", positionDelta);
            return;
        }

        // Gate 3: camera heading must agree with odometry.
        // The PnP flipped solution mirrors the heading by ~60 degrees.
        double headingDiffDeg = Math.abs(Math.toDegrees(
                AngleUnit.normalizeRadians(cameraPose.getHeading() - odoPose.getHeading())
        ));
        if (headingDiffDeg > FUSION_MAX_HEADING_DIFF_DEG) {
            consecutiveGoodFrames = 0;
            status = String.format("WARNING: heading gap %.1f° — fusion blocked, press R-stick to hard-reset", headingDiffDeg);
            return;
        }

        // All gates passed — accumulate toward required consensus.
        consecutiveGoodFrames++;
        if (consecutiveGoodFrames < FUSION_MIN_FRAMES) {
            status = String.format("Building consensus %d/%d", consecutiveGoodFrames, FUSION_MIN_FRAMES);
            return;
        }

        // Soft correction: nudge odometry ALPHA=15% toward camera each frame.
        // Gradual blending means camera noise causes slow drift, not sudden jumps.
        double correctedX = odoPose.getX() + FUSION_ALPHA * (cameraPose.getX() - odoPose.getX());
        double correctedY = odoPose.getY() + FUSION_ALPHA * (cameraPose.getY() - odoPose.getY());
        double correctedHeading = AngleUnit.normalizeRadians(odoPose.getHeading() + FUSION_ALPHA *
                AngleUnit.normalizeRadians(cameraPose.getHeading() - odoPose.getHeading()));

        drivetrain.setPose(new Pose(correctedX, correctedY, correctedHeading));
        status = String.format("Auto-fusing (Δpos=%.1f in, Δhdg=%.1f°)", positionDelta, headingDiffDeg);
    }

    /**
     * Emergency hard reset: immediately snaps odometry to camera pose,
     * bypassing the gradual fusion. Use when driver knows the camera
     * has a clean, direct view of the AprilTag.
     */
    public boolean resetPose(Drivetrain drivetrain) {
        Pose cameraPose = getRobotPoseFromCamera();
        if (cameraPose == null) return false;

        if (!isReasonableReset(drivetrain.getPose(), cameraPose)) {
            return false;
        }

        drivetrain.setPose(cameraPose);
        consecutiveGoodFrames = 0;
        status = String.format(
                "Hard reset → x=%.1f y=%.1f h=%.1f°",
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
            status = String.format("Need %d tags, saw %d", Constants.limelightMinimumTagCount, tagCount);
            return null;
        }

        return convertToPedroPose(result.getBotpose());
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

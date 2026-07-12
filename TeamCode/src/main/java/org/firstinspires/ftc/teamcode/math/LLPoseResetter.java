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
    // FUSION_TIME_CONSTANT_S: how quickly odometry is pulled toward the camera,
    // expressed as a time constant (seconds) rather than a per-frame percent.
    // The blend factor is derived each frame as alpha = 1 - e^(-dt/tau), so the
    // real-world correction speed is the SAME regardless of camera frame rate
    // (15, 30 or 50 Hz). ~1 s converges fast enough to settle the aim between
    // shots while still averaging out per-frame AprilTag noise. Smaller =
    // snappier, larger = gentler/smoother. Retune on hardware if aim twitches
    // (raise toward 1.5) or corrects too slowly before a shot (lower toward 0.5).
    private static final double FUSION_TIME_CONSTANT_S = 1.0;
    // Require this many consecutive valid frames before any correction fires.
    // At ~15 Hz camera update rate, 3 frames = ~200ms consensus window.
    private static final int FUSION_MIN_FRAMES = 3;
    // Robot must be moving slower than this for camera readings to be usable.
    // At 8 in/s and ~150ms camera latency, positional error from motion is ~1.2 in.
    // This gate only sees REAL motion because Drivetrain.setPose() drops the
    // velocity sample on a teleport, so fusion corrections can't self-trip it.
    private static final double FUSION_MAX_VELOCITY_IPS = 8.0;
    // Reject camera pose if it puts the robot more than this far from odometry.
    // Catches the PnP "wrong solution" which typically places the robot 36-60 in off.
    private static final double FUSION_MAX_POSITION_JUMP_IN = 30.0;
    // Reject camera pose if its heading disagrees with odometry by more than this.
    // The PnP flipped solution usually differs by ~60 degrees; this gate catches it.
    private static final double FUSION_MAX_HEADING_DIFF_DEG = 25.0;

    private final Limelight3A camera;
    private String status = "Waiting";
    private int consecutiveGoodFrames = 0;
    // Timestamp (seconds) of the most recent valid frame read from the camera,
    // and of the last frame actually consumed by the fusion logic. Used to make
    // the consensus counter and the blend advance once per *camera* frame
    // (~15 Hz) instead of once per *loop* (~50 Hz).
    private double latestFrameTimestamp = -1;
    private double lastProcessedTimestamp = -1;

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
        // Feed the freshest robot yaw first so MegaTag2 returns a stable pose.
        updateOrientation(drivetrain);

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

        // Frame-freshness gate: the velocity/position/heading checks above run
        // every loop (~50 Hz) so a bad reading still resets consensus instantly,
        // but the Limelight only produces a new pose at ~15 Hz. Without this
        // guard the same frame would be counted 3-4x and the 15% blend would
        // compound into a near-snap onto a single (possibly noisy) frame. Only
        // advance consensus and apply a correction when the frame is actually new.
        if (latestFrameTimestamp == lastProcessedTimestamp) {
            return;
        }
        // Time since the previously consumed frame, captured before advancing
        // lastProcessedTimestamp so the time-based blend below can use it.
        double frameDt = latestFrameTimestamp - lastProcessedTimestamp;
        lastProcessedTimestamp = latestFrameTimestamp;

        // All gates passed — accumulate toward required consensus.
        consecutiveGoodFrames++;
        if (consecutiveGoodFrames < FUSION_MIN_FRAMES) {
            status = String.format("Building consensus %d/%d", consecutiveGoodFrames, FUSION_MIN_FRAMES);
            return;
        }

        // Soft correction: blend odometry toward the camera using a frame-rate-
        // independent factor. alpha = 1 - e^(-dt/tau) gives the same real-world
        // correction speed whether the camera runs at 15 or 50 Hz, so changing
        // the vision pipeline FPS can't silently make fusion more aggressive.
        // Gradual blending means camera noise causes slow drift, not sudden jumps.
        double alpha = 1.0 - Math.exp(-frameDt / FUSION_TIME_CONSTANT_S);
        double correctedX = odoPose.getX() + alpha * (cameraPose.getX() - odoPose.getX());
        double correctedY = odoPose.getY() + alpha * (cameraPose.getY() - odoPose.getY());
        double correctedHeading = AngleUnit.normalizeRadians(odoPose.getHeading() + alpha *
                AngleUnit.normalizeRadians(cameraPose.getHeading() - odoPose.getHeading()));

        drivetrain.setPose(new Pose(correctedX, correctedY, correctedHeading));
        status = String.format("Auto-fusing (Δpos=%.1f in, Δhdg=%.1f°)", positionDelta, headingDiffDeg);
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

    /**
     * Emergency hard reset: immediately snaps odometry to camera pose,
     * bypassing the gradual fusion. Use when driver knows the camera
     * has a clean, direct view of the AprilTag.
     */
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
            status = String.format(
                    "Need %d tags, saw %d",
                    Constants.limelightMinimumTagCount,
                    tagCount
            );
            return null;
        }

        // getTimestamp() is a Limelight-local timestamp in MILLISECONDS; convert
        // to seconds so frameDt matches the units of FUSION_TIME_CONSTANT_S in the
        // alpha = 1 - e^(-frameDt / tau) blend. Without this, frameDt is ~1000x too
        // large and alpha collapses to ~1.0 (a hard snap instead of a gentle blend).
        latestFrameTimestamp = result.getTimestamp() / 1000.0;
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

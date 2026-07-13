package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.HardwareNames;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.List;

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

    // Snapshot-reset sampler state. Each buffered sample is {x, y, wallClockMs}.
    private final ArrayDeque<double[]> sampleBuffer = new ArrayDeque<>();
    private long lastResetMs = 0;
    // Robot heading rate, derived from successive odometry headings, used to reject
    // frames taken while the robot is spinning (a rear camera smears badly then).
    private double lastHeadingRad = Double.NaN;
    private long lastHeadingTimeNs = 0;
    private double angularVelocityDps = 0;
    // Distance (inches) to the nearest tag on the last read frame, for telemetry.
    private double lastTagDistanceInches = -1;
    // Comma-separated IDs seen on the last read frame, for telemetry/tuning.
    private String lastSeenTagIds = "-";
    // Set true for one poll after each successful reset so the opmode can react
    // (e.g. rumble the gamepad); cleared by consumeResetEvent().
    private boolean resetEvent = false;

    public LLPoseResetter(HardwareMap hardwareMap) {
        camera = hardwareMap.get(Limelight3A.class, HardwareNames.limelight);
    }

    public void start() {
        camera.setPollRateHz(Constants.limelightPollRateHz);
        if (Constants.limelightForcePipeline) {
            camera.pipelineSwitch(Constants.limelightPipelineIndex);
        }
        camera.start();
    }

    public void stop() {
        camera.stop();
    }

    /**
     * Called every loop iteration. Dispatches to either the discrete multi-frame
     * snapshot reset (default) or the legacy continuous soft-fusion, depending on
     * Constants.limelightUseSnapshotReset. Safe to call unconditionally — does
     * nothing when the camera is unavailable or the reading looks suspect.
     */
    public void periodicUpdate(Drivetrain drivetrain) {
        // Feed the freshest robot yaw first so MegaTag2 returns a stable pose.
        updateOrientation(drivetrain);
        updateAngularVelocity(drivetrain);

        if (Constants.limelightUseSnapshotReset) {
            snapshotUpdate(drivetrain);
        } else {
            softFusionUpdate(drivetrain);
        }
    }

    /**
     * Legacy continuous soft-fusion path (fallback; enabled by setting
     * Constants.limelightUseSnapshotReset = false). Blends odometry gently toward
     * the camera every frame once FUSION_MIN_FRAMES consecutive gates pass.
     */
    private void softFusionUpdate(Drivetrain drivetrain) {
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

    // ---- Multi-frame snapshot reset (MT2 position-only, auto-triggered) --------
    // Collects several distinct, quality-gated camera frames, requires them to
    // agree, then snaps X/Y to their median while KEEPING the IMU/odometry heading
    // (MegaTag2 cannot independently measure heading). Built for infrequent,
    // deliberate looks at a tag rather than continuous blending.
    private void snapshotUpdate(Drivetrain drivetrain) {
        long nowMs = System.currentTimeMillis();

        // Cooldown so one good look does not fire repeatedly.
        if (nowMs - lastResetMs < Constants.limelightResetCooldownMs) {
            status = "Reset cooldown";
            return;
        }

        // Motion gates first — a moving/spinning robot smears the reading. These
        // clear the buffer so consensus only ever forms from a settled robot.
        double linVel = drivetrain.getVelocityInchesPerSecond();
        if (linVel > FUSION_MAX_VELOCITY_IPS) {
            sampleBuffer.clear();
            status = String.format("Moving %.1f in/s — reset paused", linVel);
            return;
        }
        if (angularVelocityDps > Constants.limelightMaxAngularVelocityDps) {
            sampleBuffer.clear();
            status = String.format("Turning %.0f°/s — reset paused", angularVelocityDps);
            return;
        }

        // Camera-side gates (valid, tag count, staleness, distance).
        LLResult result = getGatedResult();
        if (result == null) {
            pruneStaleSamples(nowMs);
            return;
        }

        // Only consume a genuinely new camera frame (LL ~15 Hz vs loop ~50 Hz), else
        // the same frame would be counted several times and fake a false consensus.
        if (latestFrameTimestamp == lastProcessedTimestamp) {
            return;
        }
        lastProcessedTimestamp = latestFrameTimestamp;

        Pose3D mt2 = result.getBotpose_MT2();
        if (mt2 == null) {
            status = "No MT2 botpose";
            return;
        }
        Pose cameraPose = convertToPedroPose(mt2);

        sampleBuffer.addLast(new double[]{cameraPose.getX(), cameraPose.getY(), nowMs});
        pruneStaleSamples(nowMs);

        int needed = Math.max(1, Constants.limelightResetSampleCount);
        if (sampleBuffer.size() < needed) {
            status = String.format("Collecting %d/%d", sampleBuffer.size(), needed);
            return;
        }

        // Agreement: reject if the buffered samples disagree (outlier / PnP flip).
        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        double[] xs = new double[sampleBuffer.size()];
        double[] ys = new double[sampleBuffer.size()];
        int i = 0;
        for (double[] s : sampleBuffer) {
            xs[i] = s[0];
            ys[i] = s[1];
            i++;
            minX = Math.min(minX, s[0]);
            maxX = Math.max(maxX, s[0]);
            minY = Math.min(minY, s[1]);
            maxY = Math.max(maxY, s[1]);
        }
        double spread = Math.max(maxX - minX, maxY - minY);
        if (spread > Constants.limelightMaxSampleSpreadInches) {
            sampleBuffer.pollFirst(); // drop oldest, keep trying with fresher frames
            status = String.format("Samples disagree %.1f in — waiting", spread);
            return;
        }

        // Median tolerates a single wild frame better than a mean.
        double medX = median(xs);
        double medY = median(ys);
        Pose odoPose = drivetrain.getPose();
        // MT2 position-only: correct X/Y, keep the IMU/odometry heading.
        Pose candidate = new Pose(medX, medY, odoPose.getHeading());

        if (!isReasonableSnapshot(odoPose, candidate)) {
            sampleBuffer.clear(); // status already set by isReasonableSnapshot
            return;
        }

        drivetrain.setPose(candidate);
        lastResetMs = nowMs;
        resetEvent = true;
        sampleBuffer.clear();
        status = String.format("Snapshot reset → x=%.1f y=%.1f (spread %.1f in)", medX, medY, spread);
    }

    // Absolute field-bounds + generous jump sanity for the snapshot path. Uses
    // field-bounds (not just proximity to odometry) as the primary check so a large
    // but well-agreed correction after real drift is allowed through, while an
    // off-field wrong solution is still rejected.
    private boolean isReasonableSnapshot(Pose odoPose, Pose candidate) {
        if (candidate.getX() < Constants.limelightFieldMinInches
                || candidate.getX() > Constants.limelightFieldMaxInches
                || candidate.getY() < Constants.limelightFieldMinInches
                || candidate.getY() > Constants.limelightFieldMaxInches) {
            status = String.format("Off-field %.0f,%.0f — rejected", candidate.getX(), candidate.getY());
            return false;
        }
        if (Constants.limelightAllowLargePoseReset) {
            return true;
        }
        double jump = Math.hypot(
                candidate.getX() - odoPose.getX(),
                candidate.getY() - odoPose.getY()
        );
        if (jump > Constants.limelightSnapshotMaxJumpInches) {
            status = String.format("Jump %.0f in > %.0f — rejected", jump, Constants.limelightSnapshotMaxJumpInches);
            return false;
        }
        return true;
    }

    private void pruneStaleSamples(long nowMs) {
        while (!sampleBuffer.isEmpty()
                && nowMs - (long) sampleBuffer.peekFirst()[2] > Constants.limelightSampleWindowMs) {
            sampleBuffer.pollFirst();
        }
    }

    // Camera-side quality gates shared by the snapshot path. Returns the raw result
    // (and advances latestFrameTimestamp) only when the frame is worth using.
    private LLResult getGatedResult() {
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

        long staleness = result.getStaleness();
        if (staleness > Constants.limelightMaxStalenessMs) {
            status = String.format("Stale frame %d ms", staleness);
            return null;
        }

        // Trust filter: only localize from GOAL tags (20/24), never the randomized,
        // off-field OBELISK motif tags (21/22/23). Records the seen IDs for telemetry.
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        boolean hasGoalTag = false;
        StringBuilder ids = new StringBuilder();
        if (fiducials != null) {
            for (LLResultTypes.FiducialResult f : fiducials) {
                int id = f.getFiducialId();
                if (ids.length() > 0) {
                    ids.append(",");
                }
                ids.append(id);
                if (id == Constants.limelightBlueGoalTagId || id == Constants.limelightRedGoalTagId) {
                    hasGoalTag = true;
                }
            }
        }
        lastSeenTagIds = ids.length() > 0 ? ids.toString() : "-";
        if (Constants.limelightRequireGoalTag && !hasGoalTag) {
            status = String.format("No goal tag (saw %s)", lastSeenTagIds);
            return null;
        }

        double distIn = nearestTagDistanceInches(result);
        lastTagDistanceInches = distIn;
        // distIn < 0 means per-tag 3D pose was unavailable — skip the distance gate
        // rather than silently rejecting every frame.
        if (distIn >= 0) {
            if (distIn < Constants.limelightMinTagDistanceInches) {
                status = String.format("Tag too close %.0f in", distIn);
                return null;
            }
            if (distIn > Constants.limelightMaxTagDistanceInches) {
                status = String.format("Tag too far %.0f in", distIn);
                return null;
            }
        }

        latestFrameTimestamp = result.getTimestamp() / 1000.0;
        return result;
    }

    // Straight-line distance (inches) to the nearest detected tag, or -1 if the
    // per-tag camera-space pose is unavailable (e.g. Full 3D disabled).
    private double nearestTagDistanceInches(LLResult result) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) {
            return -1;
        }
        double best = -1;
        for (LLResultTypes.FiducialResult f : fiducials) {
            Pose3D tagInCamera = f.getTargetPoseCameraSpace();
            if (tagInCamera == null) {
                continue;
            }
            Position p = tagInCamera.getPosition().toUnit(DistanceUnit.INCH);
            double d = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (d <= 0) {
                continue;
            }
            if (best < 0 || d < best) {
                best = d;
            }
        }
        return best;
    }

    // Track heading rate from successive odometry headings. A pose reset keeps the
    // same heading (MT2 position-only), so this can never self-trip on a reset.
    private void updateAngularVelocity(Drivetrain drivetrain) {
        Pose pose = drivetrain.getPose();
        long now = System.nanoTime();
        if (pose != null && !Double.isNaN(lastHeadingRad)) {
            double dt = (now - lastHeadingTimeNs) / 1e9;
            if (dt >= 0.005 && dt < 0.5) {
                double dHeading = AngleUnit.normalizeRadians(pose.getHeading() - lastHeadingRad);
                angularVelocityDps = Math.abs(Math.toDegrees(dHeading)) / dt;
            }
        }
        if (pose != null) {
            lastHeadingRad = pose.getHeading();
            lastHeadingTimeNs = now;
        }
    }

    private static double median(double[] values) {
        double[] copy = values.clone();
        Arrays.sort(copy);
        int n = copy.length;
        if (n == 0) {
            return 0;
        }
        return (n % 2 == 1) ? copy[n / 2] : (copy[n / 2 - 1] + copy[n / 2]) / 2.0;
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

    // True exactly once after each successful reset. The opmode polls this to give
    // the driver feedback (e.g. a gamepad rumble) that the re-localization landed.
    public boolean consumeResetEvent() {
        boolean event = resetEvent;
        resetEvent = false;
        return event;
    }

    // Compact live-tuning readout: nearest-tag distance, robot turn rate, and how
    // many frames are currently buffered toward a snapshot reset.
    public String getDebug() {
        return String.format("tags=%s dist=%.0fin angVel=%.0f°/s samples=%d",
                lastSeenTagIds, lastTagDistanceInches, angularVelocityDps, sampleBuffer.size());
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

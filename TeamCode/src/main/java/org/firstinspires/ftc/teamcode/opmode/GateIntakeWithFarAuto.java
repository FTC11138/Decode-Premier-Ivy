package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.math.TractorBeam;
import org.firstinspires.ftc.teamcode.math.TurretLocation;
import org.firstinspires.ftc.teamcode.math.WaveLength;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.util.Constants;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.deadline;

/**
 * Shared logic for the "GateIntakeWithFar" autonomous.
 *
 * Sequence:
 *   1. Drive from the start pose -> (74,72) and fire the preload when ready.
 *   2. Second row: sweep-intake the curve out to (16,62) and back to (57,78)
 *      (throttled in the ball zone), settle, then shoot.
 *   3. Gate x2: drive out to the gate and intake - leave immediately at 3 balls,
 *      500 ms after reaching 2, or after a 1.5 s fallback (so 0-1 balls still
 *      leaves). The first cycle returns to the shoot start (57,78) so it repeats
 *      cleanly; the last returns to (57,88). Each gate drive is timeout-guarded so
 *      a follower that doesn't quite reach the gate can't leave the robot stuck.
 *   4. First row: sweep-intake out to (18,88) and back to (47,88), settle,
 *      shoot, then park at (26,88).
 *
 * All post-preload x-coordinates were shifted -4 in authored x; the preload location
 * (x=64) is the reference and unchanged.
 *
 * Shoot poses were shifted +6 in authored x from an earlier revision; each Bezier
 * control point adjacent to a shoot pose was shifted with it to keep the approach
 * tangent (angle + speed) into/out of the shoot pose unchanged.
 *
 * The turret pre-aims at the fixed upcoming shoot pose while driving (so it holds
 * steady instead of swinging as the live pose changes), then live-corrects to the
 * actual pose once within AIM_LIVE_CORRECT_DISTANCE of the shot. The flywheel
 * stays spun up (interpolated by position). Robot pose, turret angle, and
 * Alliance.current all carry into TeleOp automatically.
 *
 * Coordinates are authored for BLUE (left side); RED mirrors them across the field.
 */
public abstract class GateIntakeWithFarAuto extends RobotOpMode {
    private static final double FIELD_WIDTH = 141.5;
    // Fallback: if we only ever get 0-1 balls at the gate, leave after this so the
    // routine always moves on (never sits stuck when balls just aren't coming).
    private static final long GATE_INTAKE_TIMEOUT_MS = 1000;
    // Target is at least 2 balls: once we reach 2, give a 3rd this long to seat,
    // then leave. REPLACES the fallback timeout - it is not added on top of it.
    private static final long GATE_TWO_BALL_SETTLE_MS = 500;
    // Safety net so a follower that never settles (didn't quite reach the gate
    // position) can't leave the robot stuck: the gate drive gives up after this
    // and the sequence moves on. Tune to just above the real gate drive time.
    private static final long GATE_DRIVE_TIMEOUT_MS = 1500;
    // Hard cap on the gate intake wait: no matter what the ball logic says, leave
    // and head back after this so the robot can never sit stuck at the gate.
    private static final long GATE_HARD_TIMEOUT_MS = 2000;
    // On the way in to the gate, pause at the FIRST gate position (gate1X,gate1Y,
    // per-side) this long before continuing to the intake position (17,58), so the
    // gate has time to open. shootToGate is NOT split - we stop the drive on reaching
    // the first gate position, hold there, then re-follow the same path in.
    // GATE_FIRST_POS_TOLERANCE_IN is how close counts as "reached".
    private static final long GATE_FIRST_POS_HOLD_MS = 0;
    private static final double GATE_FIRST_POS_TOLERANCE_IN = 4.0;
    // Ease the drive down to this power within GATE_SLOW_DISTANCE_IN of the first gate
    // position so the robot doesn't slam into the gate on arrival.
    private static final double GATE_SLOW_DISTANCE_IN = 12.0;
    private static final double GATE_APPROACH_SLOW_POWER = 0.4;
    private static final int GATE_CYCLES = 2;
    // How long to keep waiting for the shooter to reach target before force-firing
    // anyway. The flywheel is kept spun up + interpolated the whole auto, so it's
    // usually ready within a few hundred ms of arriving; atTarget() fires the shot
    // as soon as it's ready. This is only the fallback cap. RAISE this if shots come
    // out weak (fired before the flywheel was up to speed).
    private static final long SHOOT_READY_TIMEOUT_MS = 1000;
    // Safety net for sweep drives: with the parametric-end early exit (see
    // followWithTimeout) the sweep normally finishes as soon as it reaches the end, so
    // this only caps a follower that never gets there. Must be COMFORTABLY longer than the
    // real sweep time - the out-and-back row sweeps run ~3 s at reduced power, so at 2000
    // ms the timeout killed the follow around the far turnaround, before the return leg,
    // and the robot shot from the wrong spot. Kept well above the true sweep duration.
    private static final long SWEEP_TIMEOUT_MS = 4000;
    // Fixed wait at the end of each sweep (still intaking) before shooting, so the robot
    // settles onto the shoot pose. The reversed sweep arrives with momentum and
    // shootWhenReady only checks position (not speed), so firing immediately threw off
    // accuracy; this wait fixes that. Applies to both rows.
    private static final long PICKUP_END_WAIT_MS = 500;
    // Dwell at the far turnaround of the LAST row (still intaking) after driving in,
    // before reversing out to shoot, so the deep-end balls seat before we leave.
    private static final long LAST_ROW_TURNAROUND_WAIT_MS = 150;
    // Post-shot wait cap: after firing we wait for the spindexer to finish its ~480-deg
    // volley turn (waitUntil !isMoving), but only up to this long. The motor completes
    // the turn on its own in the periodic, and the next cycle's auto-index already waits
    // for !isMoving, so we don't need to block for the whole mechanical rotation - just
    // long enough for the artifacts to launch. This cap lets the routine start the next
    // drive while the tail of the rotation finishes, instead of sitting idle. Raise it if
    // the last shot of a volley goes wide (i.e. it's driving off before the ball is out).
    private static final long SHOOT_ROTATE_TIMEOUT_MS = 1000;
    // Terminal (last) shot only: parking doesn't need to wait out the whole volley turn -
    // the balls launch early and the spindexer finishes its turn on its own while the robot
    // drives off. Use a shorter post-fire wait so it heads to park sooner. Raise it toward
    // SHOOT_ROTATE_TIMEOUT_MS if the last ball of the final volley starts going wide.
    private static final long LAST_SHOT_POSTFIRE_WAIT_MS = 500;
    // Shot gating (replaces the old fixed pre-delays): fire only once the robot is
    // within SHOOT_POSE_TOLERANCE_IN of the shoot pose AND the turret is within
    // TURRET_AIM_TOLERANCE_DEG of its aim target. That stops the random early shot
    // (firing while the turret was still swinging). No wait - fires as soon as those
    // (plus the flywheel) line up.
    private static final double SHOOT_POSE_TOLERANCE_IN = 5.0;
    private static final double TURRET_AIM_TOLERANCE_DEG = 5.0;
    // RED only: after the 2nd-row sweep, rotate to field-0 heading before firing (the
    // mirrored tangent arrival heading would swing the turret past its limit, same reason
    // the RED preload faces 0). Wait for the turn to settle within this tolerance, capped
    // by the timeout so it always advances.
    private static final double TURN_TO_ZERO_TOLERANCE_DEG = 4.0;
    private static final long TURN_TO_ZERO_TIMEOUT_MS = 1200;
    // Row sweeps: throttle the drivetrain to this fraction of full power while the
    // robot's (BLUE-authored) x is inside the ball (slow) zone, so it doesn't blow
    // past the balls it's collecting. Applies to both rows.
    private static final double ROW_SLOW_POWER = 2.4 / 3.0;
    private static final double ROW_SLOW_MIN_X = 22;
    private static final double ROW_SLOW_MAX_X = 40;
    // Cruise cap for BOTH row-pickup sweeps outside the ball zone, so the whole
    // pickup runs at 0.7 power for cleaner collection (was full 1.0). The ball zone
    // above is a touch slower still (ROW_SLOW_POWER). Restored to 1.0 after each
    // pickup so the gate/preload/park drives stay full speed.
    private static final double ROW_PICKUP_MAX_POWER = 0.7;
    // Run the whole auto above the default priority (0). The Spindexer periodic
    // schedules jam-recovery commands that require the same spindexer/intake motors
    // this Sequential holds; at equal priority those would OVERRIDE and kill the
    // auto mid-run. Higher priority makes them get blocked/cancelled instead.
    private static final int AUTO_PRIORITY = 1;
    // The turret pre-aims at the fixed upcoming shoot pose while driving (so it
    // holds steady instead of swinging as the live pose changes), and only
    // switches to live auto-aim on the ACTUAL pose once the robot is within this
    // many inches of the shot - close enough that the correction is small/fast.
    private static final double AIM_LIVE_CORRECT_DISTANCE = 3.0;
    // Flywheel pre-spin: within this many inches of the shoot pose the shooter latches to
    // the interpolated shoot-pose velocity/hood (stable, spun up on arrival) instead of
    // interpolating off the live pose. Kept below every collection sweep's far point so
    // it only engages on the final run-in to a shot - collection stays on interpolation.
    private static final double APPROACH_SPINUP_DISTANCE = 20.0;
    // Auto-only turret aim bias (negative = right, matching the TeleOp DPad-right
    // convention). Applied via Constants.turretAimOffsetDegrees during auto and
    // reset to 0 at the end so it doesn't leak into TeleOp. The preload shot needs
    // more right bias than the rest.
    private static final double FIRST_SHOT_TURRET_OFFSET_DEGREES = -8.0; // preload: pushed further RIGHT (was -5)
    private static final double REST_SHOT_TURRET_OFFSET_DEGREES = -7.0;   // 2nd-row shot
    // Gate-cycle shots: +3 (less negative) on top of REST, per the gate-shot tweak.
    private static final double GATE_SHOT_TURRET_OFFSET_DEGREES = -4.0;   // gate intake shoot cycles (REST + 3)
    // The first-row shot (after the gate cycles) keeps its own bias, offset from the
    // gate shots. Shifted with REST by the same 3-less-negative tweak (was -5).
    private static final double POST_GATE_TURRET_OFFSET_DEGREES = -2.0;   // first-row shot
    // RED: after mirroring the bias to the left, add this much MORE left. Net RED
    // offsets = +19 (first) / +8 (rest), i.e. left. See turretOffset().
    private static final double RED_TURRET_EXTRA_LEFT_DEGREES = 5.0;
    // RED only: extra bias added to the FIRST (preload) shot's offset, on top of the
    // mirrored value. Applied in init(); the offset switches to REST after the first shot,
    // so this affects the preload only. BLUE is unaffected.
    private static final double RED_FIRST_SHOT_EXTRA_DEGREES = -6.0;

    /** BLUE uses the authored coordinates; RED mirrors them across the field. */
    protected abstract Alliance alliance();

    private boolean mirror;
    private long gateIntakeStartMs;
    // When the ball count first reached 2 this gate cycle (-1 = not yet), so the
    // 300 ms settle window is timed from that moment rather than from arrival.
    private long gateTwoBallMs = -1;
    // First gate position, authored PER-ALLIANCE (set in init) because the two sides
    // were nudged differently: BLUE +3 in x, RED +2 in y. Can't share one mirrored
    // value, so it's not run through the usual BLUE-authored mirroring for those.
    private double gate1X;
    private double gate1Y;

    private PathChain startToShoot;
    private PathChain secondRowSweep;
    private PathChain secondRowSweepRed;
    private PathChain shootToGate;
    private PathChain gateToShoot;      // normal gate return -> (61,78) shoot start
    private PathChain gateToShootFinal; // last cycle's return -> (61,88) first-row start
    private PathChain firstRowIn;   // last row: sweep in to the turnaround
    private PathChain firstRowOut;  // last row: reverse out to the shoot pose
    private PathChain finalPark;

    // The pose the turret should pre-aim at for the upcoming shot. loop() aims at
    // this fixed pose while far away, then live-corrects to the real pose near it.
    private Pose aimTargetPose;

    @Override
    public void init() {
        super.init();

        mirror = alliance() == Alliance.RED;
        Alliance.current = alliance();

        // First gate position, per-side: BLUE nudged +3 in x, RED raised +1 in y.
        // (These are the authored values fed to pose(), which still mirrors x for RED,
        // so RED's x stays where it was and only y moves up.)
        gate1X = mirror ? 19 : 22;   // -4 x reach shift (was 23/26)
        gate1Y = mirror ? 69 : 68;

        robot.drivetrain.setStartingPose(pose(27.6546, 131.6139, 322.5094));
        robot.turret.setStartingAngle(Constants.turretHomedAngleDegrees);
        robot.turret.enableAutoAim();
        // Preload shot uses the larger right bias; switched to the smaller one
        // after the first shot in start(). Offset is flipped for RED (mirrored aim), plus
        // an extra RED-only nudge on the preload (RED_FIRST_SHOT_EXTRA_DEGREES).
        Constants.turretAimOffsetDegrees = turretOffset(FIRST_SHOT_TURRET_OFFSET_DEGREES)
                + (mirror ? RED_FIRST_SHOT_EXTRA_DEGREES : 0.0);

        buildPaths();
    }

    @Override
    public void start() {
        robot.spindexer.resetEncoderZero();
        robot.shooter.useInterpolation();
        robot.shooter.turnOn();

        CommandBuilder auto = instant(() -> aimTargetPose = preloadShootPose())
                .then(followWithTimeout(startToShoot, SWEEP_TIMEOUT_MS))
                .then(shootWhenReady())
                // Preload done: drop to the 2nd-row bias (flipped for RED).
                .then(instant(() -> Constants.turretAimOffsetDegrees = turretOffset(REST_SHOT_TURRET_OFFSET_DEGREES)))
                .then(instant(() -> aimTargetPose = pose(57, 78, 246)))
                // RED only: turn to field-0 before this shot (see turnToZeroBeforeShootIfRed).
                .then(secondRowSweepAutomatic())
                .then(shootWhenReady());
        // 2nd-row shot done: switch to the gate-cycle bias for the gate shots.
        auto = auto.then(instant(() -> Constants.turretAimOffsetDegrees = turretOffset(GATE_SHOT_TURRET_OFFSET_DEGREES)));
        for (int i = 0; i < GATE_CYCLES; i++) {
            boolean last = i == GATE_CYCLES - 1;
            auto = auto.then(gateCycle(
                    last ? gateToShootFinal : gateToShoot,
                    last ? pose(57, 88, 180) : pose(57, 78, 180)));
        }
        // Gate cycles done: shift the turret LEFT for the first-row shot.
        auto = auto.then(instant(() -> Constants.turretAimOffsetDegrees = turretOffset(POST_GATE_TURRET_OFFSET_DEGREES)))
                .then(instant(() -> aimTargetPose = pose(47, 88, 180)))
                .then(lastRowPickup())
                // Terminal shot: shorter post-fire wait so we head to park while the
                // spindexer finishes its turn (it completes on its own).
                .then(shootWhenReady(LAST_SHOT_POSTFIRE_WAIT_MS))
                // Last shot done: send the turret home (0) as we start driving off,
                // and clear the auto-only aim offset so it doesn't leak into TeleOp.
                .then(instant(() -> {
                    aimTargetPose = null;
                    robot.turret.disableAutoAim();
                    robot.turret.setTargetDegrees(0);
                    Constants.turretAimOffsetDegrees = 0;
                }))
                // Collection is over - stop the intake for the drive to park.
                .then(robot.intake.off())
                .then(robot.spindexer.setIntaking(false))
                .then(followWithTimeout(finalPark, SWEEP_TIMEOUT_MS));
        // Priority above the periodic-scheduled jam-recovery so it can't override
        // and kill this sequence (see AUTO_PRIORITY).
        auto.setPriority(AUTO_PRIORITY).schedule();
    }

    @Override
    public void loop() {
        Pose current = robot.drivetrain.getPose();
        if (aimTargetPose != null && current != null) {
            double dx = aimTargetPose.getX() - current.getX();
            double dy = aimTargetPose.getY() - current.getY();
            double dist = Math.hypot(dx, dy);
            // Far: hold the pre-computed angle for the shot pose (no swinging).
            // Near: live-correct to the actual pose for a small, fast fix.
            TractorBeam.aimTurret(dist <= AIM_LIVE_CORRECT_DISTANCE ? current : aimTargetPose,
                    robot, Alliance.current);
            // Flywheel: pure live interpolation while collecting (far from the shot), but
            // once inside APPROACH_SPINUP_DISTANCE latch it to the interpolated shoot-pose
            // value so it is stable and already at speed on arrival instead of chasing a
            // still-rising target across the last stretch. The latched value is the SAME
            // WaveLength interpolation value the shot uses, so the shot speed is unchanged
            // - only the tail of the approach spins up eagerly. Collection speed is left
            // fully on interpolation (APPROACH_SPINUP_DISTANCE is inside every collection
            // sweep's far point, so it only ever engages on the final run-in to a shot).
            if (dist <= APPROACH_SPINUP_DISTANCE) {
                Pose turretPose = TurretLocation.getTurretPose(aimTargetPose);
                robot.shooter.setTarget(WaveLength.getVelocityWithInterpolation(turretPose, Alliance.current));
                robot.shooter.setHoodPosition(WaveLength.getHoodWithInterpolation(turretPose, Alliance.current));
            } else {
                robot.shooter.useInterpolation();
            }
        } else if (robot.turret.autoAimEnabled && current != null) {
            TractorBeam.aimTurret(current, robot, Alliance.current);
        }

        super.loop();
    }

    /**
     * Follow a path, finishing as soon as the robot reaches the path's parametric end
     * OR timeoutMs elapses. The parametric-end check advances the routine the instant
     * the robot arrives instead of sitting out the timeout while the follower settles -
     * that settle wait is what made it look "stuck" between paths. Every path drive
     * goes through this, so all of them are timeout-guarded.
     */
    private Command followWithTimeout(PathChain path, long timeoutMs) {
        return robot.drivetrain.followPath(path)
                .raceWith(waitUntil(() -> robot.drivetrain.follower.atParametricEnd()))
                .raceWith(waitMs(timeoutMs));
    }

    /**
     * A no-op that COMPLETES immediately. Do NOT use Command.NOOP as a conditional
     * branch: its done() is a constant false, so a Sequential waits on it forever -
     * that hangs the gate return and the 2nd-row shot whenever ballCount < 3.
     */
    private static Command noOp() {
        return instant(() -> {});
    }

    private Command gateCycle(PathChain returnPath, Pose shootPose) {
        return instant(() -> aimTargetPose = shootPose)
                .then(robot.intake.on())
                .then(robot.spindexer.setIntaking(true))
                // Drive toward the gate but PAUSE at the first gate position for a
                // beat so the gate can finish opening, then continue in to the intake
                // position. shootToGate stays one pathchain: we stop the drive on
                // reaching the first gate position, hold there, then re-follow it in.
                // The drive eases down to a gentle power within GATE_SLOW_DISTANCE_IN
                // of the gate so it doesn't slam into it.
                .then(deadline(
                        robot.drivetrain.followPath(shootToGate)
                                .raceWith(waitMs(GATE_DRIVE_TIMEOUT_MS), waitUntil(this::nearFirstGate)),
                        slowNearGate1Control()))
                .then(instant(() -> robot.drivetrain.follower.setMaxPower(1.0)))
                .then(instant(() -> robot.drivetrain.follower.holdPoint(robot.drivetrain.getPose())))
                .then(waitMs(GATE_FIRST_POS_HOLD_MS))
                .then(followWithTimeout(shootToGate, GATE_DRIVE_TIMEOUT_MS))
                .then(instant(() -> {
                    gateIntakeStartMs = System.currentTimeMillis();
                    gateTwoBallMs = -1;
                }))
                // Leave on the ball logic, but hard-cap it so it always heads back.
                .then(waitUntil(this::gateIntakeDone)
                        .raceWith(waitMs(GATE_HARD_TIMEOUT_MS)))
                // Keep the intake running on the way back so it keeps grabbing
                // balls - unless we're already at 3, in which case stop it now.
                .then(conditional(() -> robot.spindexer.getBallCount() >= 3,
                        robot.intake.off().then(robot.spindexer.setIntaking(false)),
                        noOp()))
                .then(followWithTimeout(returnPath, GATE_DRIVE_TIMEOUT_MS))
                // Keep intaking through the shot so we keep grabbing balls; the
                // mid-cycle check above already stopped it if we hit 3. Do NOT force
                // it off here - the shoot rotation pauses auto-indexing on its own.
                .then(shootWhenReady());
    }

    /**
     * Gate-intake exit rules, evaluated every loop while we sit at the gate:
     *   3 balls   -> leave immediately (go back to shoot).
     *   2 balls   -> leave GATE_TWO_BALL_SETTLE_MS after first reaching 2
     *                (does NOT wait out the fallback timeout).
     *   0-1 balls -> leave once the GATE_INTAKE_TIMEOUT_MS fallback elapses.
     */
    private boolean gateIntakeDone() {
        int ballCount = robot.spindexer.getBallCount();
        long now = System.currentTimeMillis();
        if (ballCount >= 3) {
            return true;
        }
        if (ballCount >= 2) {
            if (gateTwoBallMs < 0) {
                gateTwoBallMs = now;
            }
            return now - gateTwoBallMs >= GATE_TWO_BALL_SETTLE_MS;
        }
        gateTwoBallMs = -1;
        return now - gateIntakeStartMs >= GATE_INTAKE_TIMEOUT_MS;
    }

    /** True once the robot is within tolerance of the first gate position (per-side). */
    private boolean nearFirstGate() {
        Pose p = robot.drivetrain.getPose();
        if (p == null) return false;
        Pose g = pose(gate1X, gate1Y, 160);
        return Math.hypot(p.getX() - g.getX(), p.getY() - g.getY()) <= GATE_FIRST_POS_TOLERANCE_IN;
    }

    /** True once the robot is within SHOOT_POSE_TOLERANCE_IN of the current shoot pose. */
    private boolean nearShootPose() {
        Pose p = robot.drivetrain.getPose();
        if (p == null || aimTargetPose == null) return false;
        return Math.hypot(p.getX() - aimTargetPose.getX(), p.getY() - aimTargetPose.getY())
                <= SHOOT_POSE_TOLERANCE_IN;
    }

    /**
     * RED only: rotate in place to field-0 heading at the current (2nd-row) shoot pose
     * before firing, so the turret stays within its limit (the mirrored tangent arrival
     * heading would push it past +90). BLUE keeps its arrival heading (no-op). The turret
     * re-aims off the live pose as the robot turns, so it's aimed once the turn settles.
     */
    private Command secondRowSweepAutomatic() {
        if (!mirror) return rowPickup(secondRowSweep);
        return rowPickup(secondRowSweepRed);
//        // pose(57,78,180) on RED == the shoot position at field-0 heading (hdg(180)==0 when
//        // mirrored). Target the FIXED shoot pose (not the current pose) so the shot stays
//        // gated on actually being AT it - otherwise, if the robot isn't fully back at the
//        // shoot pose, it would fire wherever it happens to be.
//        Pose shoot = pose(57, 78, 180);
//        return instant(() -> {
//            aimTargetPose = shoot;
//            robot.drivetrain.follower.holdPoint(shoot);
//        }).then(waitUntil(() -> nearShootPose() && headingNearZero())
//                .raceWith(waitMs(TURN_TO_ZERO_TIMEOUT_MS)));
    }

    /** True once the robot heading is within TURN_TO_ZERO_TOLERANCE_DEG of field-0. */
    private boolean headingNearZero() {
        Pose p = robot.drivetrain.getPose();
        if (p == null) return false;
        double errDeg = ((Math.toDegrees(p.getHeading()) + 540) % 360) - 180;
        return Math.abs(errDeg) <= TURN_TO_ZERO_TOLERANCE_DEG;
    }

    /**
     * While running (deadline'd to the gate approach), throttle the drive to a gentle
     * power once within GATE_SLOW_DISTANCE_IN of the first gate position so the robot
     * eases in instead of slamming the gate. Full power is restored after the drive.
     */
    private Command slowNearGate1Control() {
        return infinite(() -> {
            Pose p = robot.drivetrain.getPose();
            Pose g = pose(gate1X, gate1Y, 160);
            double dist = (p == null) ? Double.MAX_VALUE
                    : Math.hypot(p.getX() - g.getX(), p.getY() - g.getY());
            robot.drivetrain.follower.setMaxPower(
                    dist <= GATE_SLOW_DISTANCE_IN ? GATE_APPROACH_SLOW_POWER : 1.0);
        });
    }

    /**
     * Turret pre-aim target for the preload. Position mirrors normally; heading is
     * field-270 (down) for BLUE but field-0 for RED - the RED-mirrored 270 would
     * swing the turret past its limit, so the robot faces 0 there (matches the RED
     * branch of startToShoot).
     */
    private Pose preloadShootPose() {
        double x = mirror ? FIELD_WIDTH - 64 : 64;
        double headingRad = mirror ? 0.0 : Math.toRadians(270);
        return new Pose(x, 72, headingRad);
    }

    /**
     * Auto turret aim bias. RED's aim is mirrored, so the BLUE (right) bias is flipped
     * to the left, PLUS an extra RED_TURRET_EXTRA_LEFT_DEGREES to the left to correct
     * for the turret on that side. So BLUE -14/-3 (right) becomes RED +19/+8 (left).
     */
    private double turretOffset(double blueOffsetDegrees) {
        return mirror ? -blueOffsetDegrees + RED_TURRET_EXTRA_LEFT_DEGREES : blueOffsetDegrees;
    }
    /**
     * Pick up a row: intake it, slowing through the ball (slow) zone. The spindexer
     * indexes on its own (normal auto-index) - no scripted turning here.
     */
    private Command rowPickup(PathChain path) {
        return robot.intake.on()
                .then(robot.spindexer.setIntaking(true))
                .then(deadline(
                        followWithTimeout(path, SWEEP_TIMEOUT_MS),
                        slowZoneControl()))
                .then(instant(() -> robot.drivetrain.follower.setMaxPower(1.0)))
                // Wait at the end of the pickup (still intaking) so the robot settles.
                .then(waitMs(PICKUP_END_WAIT_MS))
                // Keep intaking on the way to the shot so we keep grabbing balls;
                // only stop if we're already full (3 balls).
                .then(conditional(() -> robot.spindexer.getBallCount() >= 3,
                        robot.intake.off().then(robot.spindexer.setIntaking(false)),
                        noOp()));
    }

    /**
     * Like rowPickup, but for the LAST row: sweep IN, dwell at the far turnaround
     * (still intaking) so the deep-end balls seat, then reverse OUT to the shoot pose.
     * Both legs slow through the ball zone, same as rowPickup.
     */
    private Command lastRowPickup() {
        return robot.intake.on()
                .then(robot.spindexer.setIntaking(true))
                .then(deadline(
                        followWithTimeout(firstRowIn, SWEEP_TIMEOUT_MS),
                        slowZoneControl()))
                // Dwell at the deep end of the row before reversing out.
                .then(waitMs(LAST_ROW_TURNAROUND_WAIT_MS))
                .then(deadline(
                        followWithTimeout(firstRowOut, SWEEP_TIMEOUT_MS),
                        slowZoneControl()))
                .then(instant(() -> robot.drivetrain.follower.setMaxPower(1.0)))
                // Settle before the (last) shot, but key off the real readiness signal
                // instead of a blind fixed wait: proceed the moment the flywheel is at
                // target, capped at PICKUP_END_WAIT_MS. shootWhenReady still gates on pose +
                // turret, so this only removes the dead time when we're already spun up.
                .then(waitUntil(() -> robot.shooter.atTarget())
                        .raceWith(waitMs(PICKUP_END_WAIT_MS)))
                .then(conditional(() -> robot.spindexer.getBallCount() >= 3,
                        robot.intake.off().then(robot.spindexer.setIntaking(false)),
                        noOp()));
    }

    /**
         * Throttles the drivetrain to ROW_SLOW_POWER whenever the robot is in the ball
     * (slow) zone, else full power. Runs for the whole sweep (deadline'd to it).
     */
    private Command slowZoneControl() {
        return infinite(() -> robot.drivetrain.follower.setMaxPower(
                inSlowZone() ? ROW_SLOW_POWER : ROW_PICKUP_MAX_POWER));
    }

    /** True while the robot's BLUE-authored x is inside the ball (slow) zone. */
    private boolean inSlowZone() {
        Pose p = robot.drivetrain.getPose();
        if (p == null) return false;
        double authoredX = mirror ? FIELD_WIDTH - p.getX() : p.getX();
        return authoredX >= ROW_SLOW_MIN_X && authoredX <= ROW_SLOW_MAX_X;
    }

    /**
     * Wait until the shooter is at power - the exact same signal the "ready" LED
     * uses (Shooter.atTarget()) - then rotate the spindexer to fire, wait for that
     * rotation to finish, and move on. If atTarget() never trips, the shot is
     * forced after SHOOT_READY_TIMEOUT_MS; the rotation wait is bounded too, so the
     * routine always advances after a shot. No jam/retry logic here on purpose.
     */
    private Command shootWhenReady() {
        return shootWhenReady(SHOOT_ROTATE_TIMEOUT_MS);
    }

    /**
     * As shootWhenReady(), but with an explicit cap on the post-fire wait for the volley
     * to clear. The terminal shot passes a shorter cap so the routine can start driving to
     * park while the spindexer finishes its turn (see LAST_SHOT_POSTFIRE_WAIT_MS).
     */
    private Command shootWhenReady(long postFireWaitMs) {
        // Fire only once the flywheel is up to speed AND the robot is close to the
        // shoot pose AND the turret has actually aimed - so it can't fire early with
        // the turret still swinging (that was the random early shot, worst after the
        // 2nd-row sweep and the first gate return). No fixed wait; it fires the instant
        // those line up. The timeout is a safety cap so a slightly-off turret/flywheel
        // can't stall the routine.
        return waitUntil(() -> robot.shooter.atTarget()
                        && nearShootPose()
                        && robot.turret.atTarget(TURRET_AIM_TOLERANCE_DEG))
                .raceWith(waitMs(SHOOT_READY_TIMEOUT_MS))
                .then(robot.spindexer.rotateShootCW())
                .then(waitUntil(() -> !robot.spindexer.isMoving())
                        .raceWith(waitMs(postFireWaitMs)));
    }

    // ----- Paths ------------------------------------------------------------

    private void buildPaths() {
        // Same curve both sides (control stays straight above (64,72) for a vertical
        // drop in). BLUE keeps the tangential heading (ends facing ~270). RED instead
        // interpolates to a fixed heading of 0 at the preload: the RED-mirrored 270
        // heading would swing the turret past its limit, so we face field-0 there.
        if (mirror) {
            startToShoot = robot.drivetrain.follower.pathBuilder()
                    .addPath(new BezierCurve(
                            pose(27.6546, 131.6139, 322.5094),
                            pose(64, 100.65, 270),
                            pose(64, 72, 270)))
                    .setLinearHeadingInterpolation(hdg(322.5094), 0.0)
                    .build();
        } else {
            startToShoot = robot.drivetrain.follower.pathBuilder()
                    .addPath(new BezierCurve(
                            pose(27.6546, 131.6139, 322.5094),
                            pose(64, 100.65, 270),
                            pose(64, 72, 270)))
                    .setTangentHeadingInterpolation()
                    .build();
        }
        // Every x below is shifted -4 vs the pre-adjustment values (post-first-shot
        // reach adjustment). The 2nd-row far turnaround goes 20 -> 16, i.e. deeper.
        secondRowSweep = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(pose(60, 72, 270), pose(55.77, 62, 180), pose(14, 62, 180)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(pose(14, 62, 180), pose(46, 62, 180), pose(57, 78, 180)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        secondRowSweepRed = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(pose(60, 72, 270), pose(55.77, 62, 180), pose(14, 62, 180)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(pose(14, 62, 180), pose(46, 62, 180), pose(57, 78, 180)))
                .setLinearHeadingInterpolation(hdg(180), hdg(180))
                .build();
        shootToGate = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(pose(57, 78, 180), pose(45, 65, 160), pose(gate1X, gate1Y, 160)))
                .setTangentHeadingInterpolation()
                // 173.42 = tangent angle at the end of the curve above, so the
                // linear segment starts exactly where the tangent left off and
                // the robot never has to turn at the seam.
                .addPath(new BezierLine(pose(gate1X, gate1Y, 173.42), pose(11, mirror ? 57 : 55, 107)))
                .setLinearHeadingInterpolation(hdg(173.42), hdg(107))
                .build();
        // Normal gate return: back to the shoot START (57,78) so every cycle lines
        // up and the next shootToGate begins from the same place. Curved via
        // (51,55) but heading stays linear.
        gateToShoot = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(pose(11, mirror ? 57 : 55, 107), pose(51, 55, 145), pose(57, 78, 180)))
                .setLinearHeadingInterpolation(hdg(107), hdg(180))
                .build();
        // Last gate cycle only: return to (57,88), which is where the first-row
        // sweep begins.
        gateToShootFinal = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(pose(11, mirror ? 57 : 55, 107), pose(51, 55, 145), pose(57, 88, 180)))
                .setLinearHeadingInterpolation(hdg(107), hdg(180))
                .build();
        // Last row split at the far turnaround (18,88) so we can dwell there before
        // reversing out. Same waypoints/headings as the old single reversed sweep.
        firstRowIn = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(pose(57, 88, 180), pose(18, 88, 180)))
                .setTangentHeadingInterpolation()
                .build();
        firstRowOut = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(pose(18, 88, 180), pose(47, 88, 180)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        finalPark = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(pose(47, 88, 180), pose(26, 88, 180)))
                .setTangentHeadingInterpolation()
                .build();
    }

    private Pose pose(double x, double y, double headingDeg) {
        return new Pose(mirror ? FIELD_WIDTH - x : x, y, hdg(headingDeg));
    }

    private double hdg(double headingDeg) {
        double radians = Math.toRadians(headingDeg);
        return mirror ? Math.PI - radians : radians;
    }
}

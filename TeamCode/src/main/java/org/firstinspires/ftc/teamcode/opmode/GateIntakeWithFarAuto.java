package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.math.TractorBeam;
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
 *   1. Drive (26.5,130) -> (68,72) and fire the preload when the shooter is ready.
 *   2. Second row: sweep-intake the curve out to (22,62) and back to (61,78)
 *      (throttled to 2/3 power in the x=22..40 zone), wait briefly, then shoot.
 *   3. Gate x2: drive out to the gate and intake - leave immediately at 3 balls,
 *      500 ms after reaching 2, or after a 1.5 s fallback (so 0-1 balls still
 *      leaves). The first cycle returns to the shoot start (61,78) so it repeats
 *      cleanly; the last returns to (61,88). Each gate drive is timeout-guarded so
 *      a follower that doesn't quite reach the gate can't leave the robot stuck.
 *   4. First row: sweep-intake out to (22,88) and back to (48,88), wait briefly,
 *      shoot, then park at (30,88).
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
    private static final long GATE_INTAKE_TIMEOUT_MS = 1500;
    // Target is at least 2 balls: once we reach 2, give a 3rd this long to seat,
    // then leave. REPLACES the fallback timeout - it is not added on top of it.
    private static final long GATE_TWO_BALL_SETTLE_MS = 500;
    // Safety net so a follower that never settles (didn't quite reach the gate
    // position) can't leave the robot stuck: the gate drive gives up after this
    // and the sequence moves on. Tune to just above the real gate drive time.
    private static final long GATE_DRIVE_TIMEOUT_MS = 3000;
    // Hard cap on the gate intake wait: no matter what the ball logic says, leave
    // and head back after this so the robot can never sit stuck at the gate.
    private static final long GATE_HARD_TIMEOUT_MS = 2000;
    private static final int GATE_CYCLES = 2;
    // How long to keep waiting for the shooter to reach target after a path ends
    // before force-firing anyway. If it's already up to power it fires at once;
    // this just stops a too-tight readiness tolerance from stalling the routine.
    private static final long SHOOT_READY_TIMEOUT_MS = 2000;
    // Safety net for sweep drives, mirroring GATE_DRIVE_TIMEOUT_MS, so a follower
    // that won't settle at the end of a (reversed) sweep can't block the shot.
    private static final long SWEEP_TIMEOUT_MS = 5000;
    // After a pickup path ends, sit still (still intaking) this long so the last
    // ball has time to seat before we drive off to shoot.
    private static final long PICKUP_END_WAIT_MS = 300;
    // Hang-guard for the shoot rotation: once fired, wait at most this long for the
    // spindexer to finish turning before moving on, so it can never stall the auto.
    private static final long SHOOT_ROTATE_TIMEOUT_MS = 2500;
    // Brief settle right before firing (robot keeps holding position and the turret
    // keeps correcting during it, since both run in their own periodics).
    private static final long SHOOT_PRE_DELAY_MS = 100;
    // Row sweeps: throttle the drivetrain to this fraction of full power while the
    // robot's (BLUE-authored) x is inside the ball (slow) zone, so it doesn't blow
    // past the balls it's collecting. Applies to both rows.
    private static final double ROW_SLOW_POWER = 2.0 / 3.0;
    private static final double ROW_SLOW_MIN_X = 22;
    private static final double ROW_SLOW_MAX_X = 40;
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
    // Auto-only turret aim bias (negative = right, matching the TeleOp DPad-right
    // convention). Applied via Constants.turretAimOffsetDegrees during auto and
    // reset to 0 at the end so it doesn't leak into TeleOp. The preload shot needs
    // more right bias than the rest.
    private static final double FIRST_SHOT_TURRET_OFFSET_DEGREES = -14.0; // 14 deg right
    private static final double REST_SHOT_TURRET_OFFSET_DEGREES = -3.0;   // 3 deg right

    /** BLUE uses the authored coordinates; RED mirrors them across the field. */
    protected abstract Alliance alliance();

    private boolean mirror;
    private long gateIntakeStartMs;
    // When the ball count first reached 2 this gate cycle (-1 = not yet), so the
    // 300 ms settle window is timed from that moment rather than from arrival.
    private long gateTwoBallMs = -1;

    private PathChain startToShoot;
    private PathChain secondRowSweep;
    private PathChain shootToGate;
    private PathChain gateToShoot;      // normal gate return -> (61,78) shoot start
    private PathChain gateToShootFinal; // last cycle's return -> (61,88) first-row start
    private PathChain firstRowSweep;
    private PathChain finalPark;

    // The pose the turret should pre-aim at for the upcoming shot. loop() aims at
    // this fixed pose while far away, then live-corrects to the real pose near it.
    private Pose aimTargetPose;

    @Override
    public void init() {
        super.init();

        mirror = alliance() == Alliance.RED;
        Alliance.current = alliance();

        robot.drivetrain.setStartingPose(pose(27.6546, 131.6139, 322.5094));
        robot.turret.setStartingAngle(Constants.turretHomedAngleDegrees);
        robot.turret.enableAutoAim();
        // Preload shot uses the larger right bias; switched to the smaller one
        // after the first shot in start().
        Constants.turretAimOffsetDegrees = FIRST_SHOT_TURRET_OFFSET_DEGREES;

        buildPaths();
    }

    @Override
    public void start() {
        robot.spindexer.resetEncoderZero();
        robot.shooter.useInterpolation();
        robot.shooter.turnOn();

        CommandBuilder auto = instant(() -> aimTargetPose = pose(68, 72, 270))
                .then(followWithTimeout(startToShoot, SWEEP_TIMEOUT_MS))
                .then(shootWhenReady())
                // Preload done: drop to the smaller right bias for every other shot.
                .then(instant(() -> Constants.turretAimOffsetDegrees = REST_SHOT_TURRET_OFFSET_DEGREES))
                .then(instant(() -> aimTargetPose = pose(61, 78, 246)))
                .then(rowPickup(secondRowSweep))
                .then(shootWhenReady());
        for (int i = 0; i < GATE_CYCLES; i++) {
            boolean last = i == GATE_CYCLES - 1;
            auto = auto.then(gateCycle(
                    last ? gateToShootFinal : gateToShoot,
                    last ? pose(61, 88, 180) : pose(61, 78, 180)));
        }
        auto = auto.then(instant(() -> aimTargetPose = pose(48, 88, 180)))
                .then(rowPickup(firstRowSweep))
                .then(shootWhenReady())
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
            boolean nearShot = Math.hypot(dx, dy) <= AIM_LIVE_CORRECT_DISTANCE;
            // Far: hold the pre-computed angle for the shot pose (no swinging).
            // Near: live-correct to the actual pose for a small, fast fix.
            TractorBeam.aimTurret(nearShot ? current : aimTargetPose, robot, Alliance.current);
        } else if (robot.turret.autoAimEnabled && current != null) {
            TractorBeam.aimTurret(current, robot, Alliance.current);
        }

        super.loop();
    }

    /** Follow a path but never let a stuck follower stall the auto: give up after timeoutMs. */
    private Command followWithTimeout(PathChain path, long timeoutMs) {
        return robot.drivetrain.followPath(path).raceWith(waitMs(timeoutMs));
    }

    private Command gateCycle(PathChain returnPath, Pose shootPose) {
        return instant(() -> aimTargetPose = shootPose)
                .then(robot.intake.on())
                .then(robot.spindexer.setIntaking(true))
                .then(robot.drivetrain.followPath(shootToGate)
                        .raceWith(waitMs(GATE_DRIVE_TIMEOUT_MS)))
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
                        Command.NOOP))
                .then(robot.drivetrain.followPath(returnPath)
                        .raceWith(waitMs(GATE_DRIVE_TIMEOUT_MS)))
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
    /**
     * Pick up a row: intake it, slowing through the ball (slow) zone. The spindexer
     * indexes on its own (normal auto-index) - no scripted turning here.
     */
    private Command rowPickup(PathChain path) {
        return robot.intake.on()
                .then(robot.spindexer.setIntaking(true))
                .then(deadline(
                        robot.drivetrain.followPath(path).raceWith(waitMs(SWEEP_TIMEOUT_MS)),
                        slowZoneControl()))
                .then(instant(() -> robot.drivetrain.follower.setMaxPower(1.0)))
                // Wait at the end of the pickup (still intaking) so the last ball seats.
                .then(waitMs(PICKUP_END_WAIT_MS))
                // Keep intaking on the way to the shot so we keep grabbing balls;
                // only stop if we're already full (3 balls).
                .then(conditional(() -> robot.spindexer.getBallCount() >= 3,
                        robot.intake.off().then(robot.spindexer.setIntaking(false)),
                        Command.NOOP));
    }

    /**
     * Throttles the drivetrain to ROW_SLOW_POWER whenever the robot is in the ball
     * (slow) zone, else full power. Runs for the whole sweep (deadline'd to it).
     */
    private Command slowZoneControl() {
        return infinite(() -> robot.drivetrain.follower.setMaxPower(inSlowZone() ? ROW_SLOW_POWER : 1.0));
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
        return waitUntil(() -> robot.shooter.atTarget())
                .raceWith(waitMs(SHOOT_READY_TIMEOUT_MS))
                // Slight settle before firing. The scheduler waits here, but the
                // drivetrain (holding position) and turret (aiming) run in their
                // own periodics, so they keep correcting through this pause.
                .then(waitMs(SHOOT_PRE_DELAY_MS))
                .then(robot.spindexer.rotateShootCW())
                .then(waitUntil(() -> !robot.spindexer.isMoving())
                        .raceWith(waitMs(SHOOT_ROTATE_TIMEOUT_MS)));
    }

    // ----- Paths ------------------------------------------------------------

    private void buildPaths() {
        startToShoot = robot.drivetrain.follower.pathBuilder()
                // Tangential so it leaves the start at the given 322.5 deg heading
                // with no initial spin. The control point sits straight above the
                // shoot point, so the tangent starts at 322.5 deg and ends vertical
                // (270 deg) as it drops into (68,72).
                .addPath(new BezierCurve(
                        pose(27.6546, 131.6139, 322.5094),
                        pose(68, 100.65, 270),
                        pose(68, 72, 270)))
                .setTangentHeadingInterpolation()
                .build();
        secondRowSweep = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(pose(68, 72, 270), pose(63.77, 62, 180), pose(22, 62, 180)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(pose(22, 62, 180), pose(50, 62, 180), pose(61, 78, 180)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        shootToGate = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(pose(61, 78, 180), pose(49, 65, 160), pose(23, 68, 160)))
                .setTangentHeadingInterpolation()
                // 173.42 = tangent angle at the end of the curve above, so the
                // linear segment starts exactly where the tangent left off and
                // the robot never has to turn at the seam.
                .addPath(new BezierLine(pose(23, 68, 173.42), pose(17, 57, 110)))
                .setLinearHeadingInterpolation(hdg(173.42), hdg(110))
                .build();
        // Normal gate return: back to the shoot START (61,78) so every cycle lines
        // up and the next shootToGate begins from the same place. Curved via
        // (55,55) but heading stays linear.
        gateToShoot = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(pose(17, 57, 110), pose(55, 55, 145), pose(61, 78, 180)))
                .setLinearHeadingInterpolation(hdg(110), hdg(180))
                .build();
        // Last gate cycle only: return to the file's end position (61,88), which
        // is where the first-row sweep begins.
        gateToShootFinal = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(pose(17, 57, 110), pose(55, 55, 145), pose(61, 88, 180)))
                .setLinearHeadingInterpolation(hdg(110), hdg(180))
                .build();
        firstRowSweep = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(pose(61, 88, 180), pose(22, 88, 180)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(pose(22, 88, 180), pose(48, 88, 180)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        finalPark = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(pose(48, 88, 180), pose(30, 88, 180)))
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

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
 *      450 ms after reaching 2, or after a 1.4 s fallback. The first cycle returns
 *      to the shoot start (61,78) so it repeats cleanly; the last returns to
 *      (61,88). Each gate drive is timeout-guarded so a follower that won't settle
 *      can't hang the whole routine.
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
    // Fallback: how long to sit intaking at the gate if we never reach 2 balls.
    private static final long GATE_INTAKE_TIMEOUT_MS = 1400;
    // Once we reach 2 balls, give the 3rd this long to seat, then leave anyway.
    // This REPLACES the fallback timeout - it is not added on top of it.
    private static final long GATE_TWO_BALL_SETTLE_MS = 450;
    // Safety net so a follower that never settles within its end tolerances
    // (tight tolerances + predictive braking) can't hang the whole autonomous:
    // the gate drive gives up after this long and the sequence moves on. Tune to
    // just above the real gate approach/return time.
    private static final long GATE_DRIVE_TIMEOUT_MS = 3500;
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
    // Second-row sweep: throttle the drivetrain to this fraction of full power
    // while the robot's (BLUE-authored) x is inside the slow zone, so it doesn't
    // blow past the balls it's collecting.
    private static final double SECOND_ROW_SLOW_POWER = 2.0 / 3.0;
    private static final double SECOND_ROW_SLOW_MIN_X = 22;
    private static final double SECOND_ROW_SLOW_MAX_X = 40;
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
    private PathChain gateToShoot;      // normal gate return -> (57,78) shoot start
    private PathChain gateToShootFinal; // last cycle's return -> (57,88) file end
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

        robot.drivetrain.setStartingPose(pose(26.5, 130, 143));
        robot.turret.setStartingAngle(Constants.turretHomedAngleDegrees);
        robot.turret.enableAutoAim();

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
                .then(instant(() -> aimTargetPose = pose(61, 78, 246)))
                .then(secondRowPickup())
                .then(shootWhenReady());
        for (int i = 0; i < GATE_CYCLES; i++) {
            boolean last = i == GATE_CYCLES - 1;
            auto = auto.then(gateCycle(
                    last ? gateToShootFinal : gateToShoot,
                    last ? pose(61, 88, 180) : pose(61, 78, 180)));
        }
        auto = auto.then(instant(() -> aimTargetPose = pose(48, 88, 180)))
                .then(sweepIntake(firstRowSweep))
                .then(shootWhenReady())
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
                .then(waitUntil(this::gateIntakeDone))
                .then(robot.intake.off())
                .then(robot.spindexer.setIntaking(false))
                .then(robot.drivetrain.followPath(returnPath)
                        .raceWith(waitMs(GATE_DRIVE_TIMEOUT_MS)))
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
    private Command sweepIntake(PathChain path) {
        return robot.intake.on()
                .then(robot.spindexer.setIntaking(true))
                .then(robot.drivetrain.followPath(path)
                        .raceWith(waitMs(SWEEP_TIMEOUT_MS)))
                // Wait at the end of the pickup (still intaking) so the last ball seats.
                .then(waitMs(PICKUP_END_WAIT_MS))
                .then(robot.intake.off())
                .then(robot.spindexer.setIntaking(false));
    }

    /**
     * Second-row pickup: same as sweepIntake, but throttles the drivetrain to
     * SECOND_ROW_SLOW_POWER while the robot's x is in the slow zone so it doesn't
     * overrun the balls. The speed control runs in parallel with the drive
     * (deadline'd to it) and full power is restored once the drive ends.
     */
    private Command secondRowPickup() {
        return robot.intake.on()
                .then(robot.spindexer.setIntaking(true))
                .then(deadline(
                        robot.drivetrain.followPath(secondRowSweep).raceWith(waitMs(SWEEP_TIMEOUT_MS)),
                        slowZoneControl()))
                .then(instant(() -> robot.drivetrain.follower.setMaxPower(1.0)))
                .then(waitMs(PICKUP_END_WAIT_MS))
                .then(robot.intake.off())
                .then(robot.spindexer.setIntaking(false));
    }

    /**
     * While running, holds the drivetrain at SECOND_ROW_SLOW_POWER whenever the
     * robot's BLUE-authored x is within [SECOND_ROW_SLOW_MIN_X, MAX_X], else full
     * power. x is un-mirrored for RED so the zone lands in the same field spot.
     */
    private Command slowZoneControl() {
        return infinite(() -> {
            Pose p = robot.drivetrain.getPose();
            if (p == null) return;
            double authoredX = mirror ? FIELD_WIDTH - p.getX() : p.getX();
            boolean slow = authoredX >= SECOND_ROW_SLOW_MIN_X && authoredX <= SECOND_ROW_SLOW_MAX_X;
            robot.drivetrain.follower.setMaxPower(slow ? SECOND_ROW_SLOW_POWER : 1.0);
        });
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
                .then(robot.spindexer.rotateShootCW())
                .then(waitUntil(() -> !robot.spindexer.isMoving())
                        .raceWith(waitMs(SHOOT_ROTATE_TIMEOUT_MS)));
    }

    // ----- Paths ------------------------------------------------------------

    private void buildPaths() {
        startToShoot = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(pose(26.5, 130, 143), pose(68, 72, 270)))
                .setLinearHeadingInterpolation(hdg(143), hdg(270))
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
                .addPath(new BezierLine(pose(23, 68, 173.42), pose(16, 57, 110)))
                .setLinearHeadingInterpolation(hdg(173.42), hdg(110))
                .build();
        // Normal gate return: back to the shoot START (61,78) so every cycle lines
        // up and the next shootToGate begins from the same place. Curved via
        // (55,55) but heading stays linear.
        gateToShoot = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(pose(16, 57, 110), pose(55, 55, 145), pose(61, 78, 180)))
                .setLinearHeadingInterpolation(hdg(110), hdg(180))
                .build();
        // Last gate cycle only: return to the file's end position (61,88), which
        // is where the first-row sweep begins.
        gateToShootFinal = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(pose(16, 57, 110), pose(55, 55, 145), pose(61, 88, 180)))
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

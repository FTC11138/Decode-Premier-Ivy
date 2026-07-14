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
 * "Far zone" autonomous: shoot from the far launch zone, feed off the last row and
 * then cycle the corner stack three times.
 *
 * Sequence:
 *   1. Preload: the robot starts ON the far shoot point (65,8.5) and fires the
 *      preload as soon as the flywheel/turret line up (no drive first).
 *   2. Last row: drive up to the row (65,8.5)->(40.5,35.5), sweep the row
 *      (40.5,35.5)->(13.5,35.5) at HALF power while intaking, then back out along the
 *      row line and down to the shoot point (52.5,13) on a reversed tangent (stays high,
 *      no low dip) and shoot. All one continuous drive.
 *   3. Corner x3: one smooth sweeping loop - shoot up to the top (14,45), a rounded
 *      ~180 U-turn, then a fast-turning dive down to the second point (12,18.5), and a
 *      reversed-tangent glide back to the shoot point (52.5,13) and shoot. It flows (no
 *      nose-in reversal) so it does not shove balls, and arrives ~west so the turret
 *      stays in range. Driving IN toward the stack (authored x 24..14) is half power;
 *      the rest is full. Repeated CORNER_CYCLES times.
 *
 * The turret pre-aims at the fixed upcoming shoot pose while driving (so it holds
 * steady instead of swinging as the live pose changes), then live-corrects to the
 * actual pose once within AIM_LIVE_CORRECT_DISTANCE of the shot. The flywheel stays
 * spun up on the far interpolation table the whole run.
 *
 * Coordinates are authored for BLUE; RED mirrors them across the field
 * (x -> FIELD_WIDTH - x, heading -> pi - heading) via point()/pose()/hdg(), the
 * same convention used everywhere else in this codebase. Every drive is wrapped in
 * a timeout and races the follower's parametric end so a follower that won't settle
 * can never stall the routine (and arrival advances the routine immediately).
 */
public abstract class FarZoneAuto extends RobotOpMode {
    private static final double FIELD_WIDTH = 141.5;

    // Run the whole auto above the default priority (0). The spindexer/intake
    // periodics schedule jam-recovery that requires the same motors this Sequential
    // holds; at equal priority those would override and kill the auto mid-run.
    private static final int AUTO_PRIORITY = 1;

    // How many times to run the corner-stack cycle after the row shot.
    private static final int CORNER_CYCLES = 3;

    // Drive powers: full everywhere except two half-speed collection windows, both
    // applied by position on ONE continuous chain (so the follower never decelerates at
    // a seam) rather than by splitting the drive, which would cost a stop each seam.
    private static final double FULL_POWER = 1.0;
    private static final double HALF_POWER = 0.5;
    // Row slow band: the row is the only part of the row excursion above this y, so it
    // cleanly marks where to drop to half power. y is never mirrored (only x is).
    private static final double ROW_SLOW_MIN_Y = 32.0;
    // Corner slow band: while driving INWARD (toward authored x=0) through this x
    // window it drops to half power to feed the stack - i.e. the two legs that go to
    // point 4 and point 5; the way back out stays full power. Authored x, so it holds on
    // both alliances (see cornerSlowControl()).
    private static final double CORNER_SLOW_MIN_X = 14.0;
    private static final double CORNER_SLOW_MAX_X = 24.0;
    // Ignore sub-noise x changes when deciding "moving inward" so pose jitter at the
    // turnaround doesn't flicker the throttle.
    private static final double INWARD_EPSILON_IN = 0.05;

    // Turret aim bias, held for the whole run (never retargeted mid-routine). Authored
    // for BLUE and mirrored for RED (see turretOffset()) so both sides get the same
    // physical correction; positive is left in this codebase's convention. Reset to 0 at
    // the end so it can't leak into TeleOp.
    private static final double TURRET_AIM_OFFSET_DEGREES = -2.0;

    // Generous per-drive safety timeout: only fires if a follower gets stuck. Normal
    // drives finish well under it because followWithTimeout also races the follower's
    // parametric end and advances the instant the robot arrives.
    private static final long DRIVE_TIMEOUT_MS = 5000;

    // After a pickup path ends, sit still (still intaking) this long so the last ball
    // has time to seat/index before we fire.
    private static final long PICKUP_SEAT_MS = 300;

    // Shot gating: fire only once the flywheel is up to speed AND the robot is within
    // SHOOT_POSE_TOLERANCE_IN of the shoot pose AND the turret has actually aimed - so
    // it can't fire early with the turret still swinging. No fixed wait; it fires the
    // instant those line up. The timeouts are safety caps so a slightly-off
    // turret/flywheel or a stuck spindexer can never stall the routine.
    private static final long SHOOT_READY_TIMEOUT_MS = 2000;
    // The preload fires from a dead-stop flywheel (spinning up from 0 to a far-shot
    // velocity), so give it a longer ready cap before force-firing. Every later shot
    // keeps SHOOT_READY_TIMEOUT_MS because the flywheel is already near speed.
    private static final long FIRST_SHOOT_READY_TIMEOUT_MS = 3000;
    private static final long SHOOT_ROTATE_TIMEOUT_MS = 2500;
    private static final double SHOOT_POSE_TOLERANCE_IN = 5.0;
    private static final double TURRET_AIM_TOLERANCE_DEG = 5.0;

    // The turret pre-aims at the fixed upcoming shoot pose while driving, and only
    // switches to live auto-aim on the ACTUAL pose once the robot is within this many
    // inches of the shot - close enough that the correction is small/fast.
    private static final double AIM_LIVE_CORRECT_DISTANCE = 3.0;

    // Start heading = the tangent at the start of the first curve (straight up toward
    // the first control point (65,35.5)), so the robot never has to rotate to begin.
    private static final double START_HEADING_DEG = 90;
    // Heading the robot arrives at the shoot point with. Two values because the row
    // return and the corner return arrive facing different ways; the turret pre-aims at
    // the matching one so it never has to swing on arrival. Row seg 3 ends ~128 deg; the
    // corner loop is designed to end at 175 deg (facing ~west, turret-comfortable).
    private static final double SHOOT_HEADING_ROW_DEG = 128;
    private static final double SHOOT_HEADING_CORNER_DEG = 175;

    /** BLUE uses the authored coordinates; RED mirrors them across the field. */
    protected abstract Alliance alliance();

    private boolean mirror;

    private PathChain rowRun;       // segs 1-3 chained: start -> row -> shoot (one drive)
    private PathChain cornerCycle;  // segs 4-6: shoot -> corner stack -> shoot (looped)

    // The pose the turret should pre-aim at for the upcoming shot. loop() aims at this
    // fixed pose while far away, then live-corrects to the real pose near it.
    private Pose aimTargetPose;

    // Previous-loop authored x, so the corner slow zone can tell inward (toward x=0)
    // from outward. NaN until the first sample.
    private double lastAuthoredX = Double.NaN;

    @Override
    public void init() {
        super.init();

        mirror = alliance() == Alliance.RED;
        Alliance.current = alliance();

        robot.drivetrain.setStartingPose(startPose());
        robot.turret.setStartingAngle(Constants.turretHomedAngleDegrees);
        robot.turret.enableAutoAim();
        // Constant aim bias for the whole run (mirrored for RED). Held, never retargeted.
        Constants.turretAimOffsetDegrees = turretOffset();

        buildPaths();
    }

    @Override
    public void start() {
        robot.spindexer.resetEncoderZero();
        // Far launch zone: force the far flywheel/hood interpolation table.
        robot.shooter.useFarInterpolation();
        robot.shooter.turnOn();

        // ----- preload (already sitting on the shoot point) -----
        CommandBuilder auto = instant(() -> aimTargetPose = startPose())
                .then(shootWhenReady(FIRST_SHOOT_READY_TIMEOUT_MS))
                // ----- last row (segs 1-3, one continuous drive) -----
                .then(instant(() -> aimTargetPose = shootPoseRow()))
                // Intake on before the run so it is fully spun up entering the row.
                .then(intakeOn())
                // One flat-out drive; slowZoneControl throttles it to half power only
                // while in the row band, so there is no stop between approach, sweep,
                // and return. Restore full power once the drive is done.
                .then(deadline(
                        followWithTimeout(rowRun, DRIVE_TIMEOUT_MS, FULL_POWER),
                        slowZoneControl()))
                .then(instant(() -> robot.drivetrain.follower.setMaxPower(FULL_POWER)))
                .then(seatThenShoot());

        // ----- corner stack, CORNER_CYCLES times, all from the same shoot point -----
        // Pre-aim at the corner return heading for the rest of the run.
        auto = auto.then(instant(() -> aimTargetPose = shootPoseCorner()));
        for (int i = 0; i < CORNER_CYCLES; i++) {
            auto = auto
                    // rotateShootCW() cleared intaking on the last shot, so re-arm it.
                    .then(intakeOn())
                    // One continuous in-and-out drive; cornerSlowControl drops it to
                    // half power only while heading INWARD through the stack band, full
                    // power on the way back out. Restore full power once the drive ends.
                    .then(deadline(
                            followWithTimeout(cornerCycle, DRIVE_TIMEOUT_MS, FULL_POWER),
                            cornerSlowControl()))
                    .then(instant(() -> robot.drivetrain.follower.setMaxPower(FULL_POWER)))
                    .then(seatThenShoot());
        }

        // ----- done: home the turret, stop collection, stop the flywheel -----
        auto = auto
                .then(instant(() -> {
                    aimTargetPose = null;
                    robot.turret.disableAutoAim();
                    robot.turret.setTargetDegrees(0);
                    Constants.turretAimOffsetDegrees = 0;
                }))
                .then(intakeOff())
                .then(instant(() -> robot.shooter.turnOff()));

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

    // ----- command helpers --------------------------------------------------

    private Command intakeOn() {
        return robot.intake.on().then(robot.spindexer.setIntaking(true));
    }

    private Command intakeOff() {
        return robot.intake.off().then(robot.spindexer.setIntaking(false));
    }

    /**
     * Let the last collected ball seat (still intaking), stop the intake only if we
     * are already full so a wedged 4th ball can't fight it, then fire. Keeping the
     * intake running below 3 balls lets us keep grabbing right up to the shot.
     */
    private Command seatThenShoot() {
        return waitMs(PICKUP_SEAT_MS)
                .then(conditional(() -> robot.spindexer.getBallCount() >= 3,
                        intakeOff(),
                        noOp()))
                .then(shootWhenReady());
    }

    /**
     * Wait until the flywheel is up to speed AND the robot is close to the shoot pose
     * AND the turret has aimed, then rotate the spindexer to fire and wait for that
     * rotation to finish. Both waits are timeout-capped so the routine always advances.
     */
    private Command shootWhenReady() {
        return shootWhenReady(SHOOT_READY_TIMEOUT_MS);
    }

    private Command shootWhenReady(long readyTimeoutMs) {
        return waitUntil(() -> robot.shooter.atTarget()
                        && nearShootPose()
                        && robot.turret.atTarget(TURRET_AIM_TOLERANCE_DEG))
                .raceWith(waitMs(readyTimeoutMs))
                .then(robot.spindexer.rotateShootCW())
                .then(waitUntil(() -> !robot.spindexer.isMoving())
                        .raceWith(waitMs(SHOOT_ROTATE_TIMEOUT_MS)));
    }

    /**
     * Follow a path capped at maxPower, finishing as soon as the robot reaches the
     * path's parametric end OR timeoutMs elapses. The parametric-end race advances the
     * routine the instant the robot arrives instead of sitting out the follower's
     * settle. Passing maxPower per call means each leg re-asserts its own power, so the
     * half-power sweep never leaks into the full-power legs.
     */
    private Command followWithTimeout(PathChain path, long timeoutMs, double maxPower) {
        return robot.drivetrain.followPath(path, maxPower)
                .raceWith(waitUntil(() -> robot.drivetrain.follower.atParametricEnd()))
                .raceWith(waitMs(timeoutMs));
    }

    /**
     * A no-op that COMPLETES immediately. Do NOT use Command.NOOP as a conditional
     * branch: its done() is a constant false, so a Sequential would wait on it forever.
     */
    private static Command noOp() {
        return instant(() -> {});
    }

    /**
     * Throttle the drivetrain to HALF_POWER whenever the robot is in the row band, else
     * full power. Run alongside (deadline'd to) the row drive so the half-speed sweep
     * needs no separate, decelerating path segment.
     */
    private Command slowZoneControl() {
        return infinite(() -> robot.drivetrain.follower.setMaxPower(
                inRowZone() ? HALF_POWER : FULL_POWER));
    }

    /** True while the robot is in the row band (the only high-y part of the excursion). */
    private boolean inRowZone() {
        Pose p = robot.drivetrain.getPose();
        return p != null && p.getY() >= ROW_SLOW_MIN_Y;
    }

    /**
     * Corner throttle: HALF_POWER only while the robot is driving INWARD (authored x
     * decreasing, toward x=0) through the stack band [CORNER_SLOW_MIN_X, MAX_X]; full
     * power otherwise, including the whole way back out. Inward is detected from the
     * authored-x delta between loops, so it works on both alliances (x is un-mirrored to
     * the authored frame first) and only slows the approach, not the exit.
     */
    private Command cornerSlowControl() {
        return infinite(() -> {
            Pose p = robot.drivetrain.getPose();
            double power = FULL_POWER;
            if (p != null) {
                double authoredX = mirror ? FIELD_WIDTH - p.getX() : p.getX();
                boolean movingInward = !Double.isNaN(lastAuthoredX)
                        && authoredX < lastAuthoredX - INWARD_EPSILON_IN;
                if (movingInward
                        && authoredX >= CORNER_SLOW_MIN_X
                        && authoredX <= CORNER_SLOW_MAX_X) {
                    power = HALF_POWER;
                }
                lastAuthoredX = authoredX;
            }
            robot.drivetrain.follower.setMaxPower(power);
        });
    }

    /** Aim bias: authored for BLUE, negated for RED so the physical correction matches. */
    private double turretOffset() {
        return mirror ? -TURRET_AIM_OFFSET_DEGREES : TURRET_AIM_OFFSET_DEGREES;
    }

    /** True once the robot is within SHOOT_POSE_TOLERANCE_IN of the current shoot pose. */
    private boolean nearShootPose() {
        Pose p = robot.drivetrain.getPose();
        if (p == null || aimTargetPose == null) return false;
        return Math.hypot(p.getX() - aimTargetPose.getX(), p.getY() - aimTargetPose.getY())
                <= SHOOT_POSE_TOLERANCE_IN;
    }

    // ----- geometry ---------------------------------------------------------

    /** Far shoot point the preload is fired from, and where the robot starts. */
    private Pose startPose() {
        return pose(65, 8.5, START_HEADING_DEG);
    }

    /** Shoot point after the ROW return (seg 3's reversed-tangent arrival heading). */
    private Pose shootPoseRow() {
        return pose(52.5, 13, SHOOT_HEADING_ROW_DEG);
    }

    /** Shoot point after a CORNER return (seg 7's reversed-tangent arrival heading). */
    private Pose shootPoseCorner() {
        return pose(52.5, 13, SHOOT_HEADING_CORNER_DEG);
    }

    private void buildPaths() {
        // Segs 1-3 as ONE chain so the follower keeps momentum through the whole row
        // excursion (it only decelerates at the final shoot point, not at each seam):
        //   seg 1  curve  start -> row start        tangent (up, then ~180 into the row)
        //   seg 2  line   row start -> row end      tangent (swept at half power - zone)
        //   seg 3  curve  row end -> shoot point    reversed tangent (backs in clean)
        // seg 3's control is up at (35,35.5): the robot backs out along the row line (so
        // its facing lines up with seg 2 at 180, no heading jump) and stays high, only
        // dropping to the shoot point near the end - it no longer dips down the left
        // side. Arrives ~128 deg (efficient; the exact end heading is free here).
        rowRun = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(point(65, 8.5), point(65, 35.5), point(40.5, 35.5)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(point(40.5, 35.5), point(13.5, 35.5)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(point(13.5, 35.5), point(35, 35.5), point(52.5, 13)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // Corner-stack sweep, one smooth chain re-followed each cycle: go back UP to the
        // top (14,45), a rounded ~180 U-turn (allowed), then a fast-turning dive down and
        // out to the second point (12,18.5), and a clean reversed-tangent back to the shoot
        // point (arriving ~west, so the turret stays in range). Control points keep it
        // smooth; the up and return legs are tangential (chassis follows the arc), the
        // dive uses the given 135 -> -130 heading and finishes that turn FAST (endTime
        // 0.3) so it is settled before it reaches the second point. Full speed; the
        // inward zone still eases it near the stack.
        //   seg 1  curve  shoot -> top (14,45)       tangential (arrives ~135 into seg 2)
        //   seg 2  curve  top -> second (12,18.5)      linear 135 -> -130, fast (endTime .3)
        //   seg 3  curve  second -> shoot            tangential reversed (arrives ~west)
        cornerCycle = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(point(52.5, 13), point(35, 24), point(14, 45)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(point(14, 45), point(18, 33), point(12, 18.5)))
                .setLinearHeadingInterpolation(hdg(135), hdg(-130), 0.3)
                .addPath(new BezierCurve(point(12, 18.5), point(33, 15), point(52.5, 13)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    /** A geometric point (heading irrelevant to a Bezier); x mirrored for RED. */
    private Pose point(double x, double y) {
        return new Pose(mirror ? FIELD_WIDTH - x : x, y);
    }

    /** A full pose (heading matters); x and heading mirrored for RED. */
    private Pose pose(double x, double y, double headingDeg) {
        return new Pose(mirror ? FIELD_WIDTH - x : x, y, hdg(headingDeg));
    }

    private double hdg(double headingDeg) {
        double radians = Math.toRadians(headingDeg);
        return mirror ? Math.PI - radians : radians;
    }
}

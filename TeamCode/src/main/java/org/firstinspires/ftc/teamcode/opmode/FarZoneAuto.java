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
import static com.pedropathing.ivy.groups.Groups.parallel;

/**
 * "Far zone" autonomous: shoot from the far launch zone, feed off the last row and
 * then cycle the corner stack twice.
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
 * The turret auto-aims normally: every loop it tracks the goal from the live robot pose.
 * The shot still waits until the chassis has settled onto the shoot heading (chassisSettled)
 * before firing. The flywheel stays spun up on the far interpolation table the whole run.
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
    private static final int CORNER_CYCLES = 2;

    // Drive powers: full everywhere except two half-speed collection windows, both
    // applied by position on ONE continuous chain (so the follower never decelerates at
    // a seam) rather than by splitting the drive, which would cost a stop each seam.
    private static final double FULL_POWER = 1.0;
    private static final double HALF_POWER = 0.5;
    // Corner-stack feed power. Faster than the row's HALF_POWER so the cycle isn't
    // sluggish, but still throttled below full so the intake actually seats the stack.
    private static final double CORNER_INTAKE_POWER = 0.75;
    // Row slow band: the row is the only part of the row excursion above this y, so it
    // cleanly marks where to drop to half power. y is never mirrored (only x is).
    private static final double ROW_SLOW_MIN_Y = 32.0;
    // Corner slow band: while driving INWARD (toward authored x=0) through this x
    // window it drops to half power to feed the stack - i.e. the two legs that go to
    // point 4 and point 5; the way back out stays full power. Authored x, so it holds on
    // both alliances (see cornerSlowControl()).
    private static final double CORNER_SLOW_MIN_X = 14.0;
    private static final double CORNER_SLOW_MAX_X = 24.0;
    // RED only: shift the human-player (corner) cycle waypoints this far in +x (field), so
    // the robot reaches further toward the RED corner. BLUE is unaffected. Flip the sign to
    // reverse the direction. See cornerPoint().
    private static final double RED_CORNER_X_SHIFT = 4.0;
    // Ignore sub-noise x changes when deciding "moving inward" so pose jitter at the
    // turnaround doesn't flicker the throttle.
    private static final double INWARD_EPSILON_IN = 0.05;

    // Per-shot turret aim bias (degrees), authored for BLUE and mirrored for RED (see
    // turretOffset()). Convention: POSITIVE = left, NEGATIVE = right. Reset to 0 at the
    // end so it can't leak into TeleOp. Both shots are biased RIGHT; the preload the most,
    // the row/gate shots a little less right. Adjust the magnitudes from field results.
    private static final double FIRST_SHOT_TURRET_OFFSET_DEGREES = -6.0;  // preload: dialed in, keep
    private static final double REST_SHOT_TURRET_OFFSET_DEGREES = -2.0;   // row+gate: nudged left from -4
    // RED only: extra bias added on top of the mirrored offset (negative = right). Applied
    // where the offset is set, so BLUE is unaffected.
    private static final double RED_FIRST_SHOT_EXTRA_DEGREES = -5.0;      // preload: 5 more
    private static final double RED_REST_SHOT_EXTRA_DEGREES = -2.0;       // row+gate: 2 more

    // Generous per-drive safety timeout: only fires if a follower gets stuck. Normal
    // drives finish well under it because followWithTimeout also races the follower's
    // parametric end and advances the instant the robot arrives.
    private static final long DRIVE_TIMEOUT_MS = 5000;

    // At the end of each pickup/cycle path the robot sits still (still intaking) this
    // long before firing, so it fully settles onto the shoot pose and the last ball
    // seats/indexes.
    private static final long PICKUP_SEAT_MS = 250;
    // On arriving at a shoot pose (end of a return path), reverse the intake this long to
    // spit out any 4th artifact wedged at the intake, so we never fire with 4 loaded.
    private static final long EJECT_REVERSE_MS = 300;
    // Pause at the end of the row collection sweep (still intaking) so the swept artifacts
    // seat before the robot backs out to shoot.
    private static final long ROW_SWEEP_END_WAIT_MS = 300;

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
    // After firing, wait for the spindexer to finish its rotation - i.e. the whole volley
    // has cleared - before moving, capped so a stuck spindexer can't stall the routine.
    private static final long SHOOT_FIRE_TIMEOUT_MS = 1500;
    private static final double SHOOT_POSE_TOLERANCE_IN = 5.0;
    private static final double TURRET_AIM_TOLERANCE_DEG = 5.0;
    // The shot waits until the chassis has settled to within this many degrees of the
    // shoot heading (chassisSettled()) instead of firing while the chassis is still turning.
    private static final double HEADING_SETTLE_TOLERANCE_DEG = 6.0;
    // Flywheel pre-spin: within this many inches of the shoot pose the shooter latches to
    // the interpolated shoot-pose (far) velocity/hood instead of interpolating off the
    // live pose. The shoot pose is the FARTHEST point in the routine, so the flywheel
    // would otherwise be spun DOWN from collecting closer in and have to spin back up on
    // arrival; latching on the run-in gets it up to speed before the shot. Kept below the
    // corner far points (~40"), so it only engages on the final approach - collection
    // stays on live far interpolation.
    private static final double APPROACH_SPINUP_DISTANCE = 30.0;

    // Start heading = the tangent at the start of the first curve (straight up toward
    // the first control point (65,35.5)), so the robot never has to rotate to begin.
    private static final double START_HEADING_DEG = -180;
    // Heading the robot arrives at the shoot point with. Two values because the row
    // return and the corner return arrive facing different ways; the turret pre-aims at
    // the matching one so it never has to swing on arrival. Row seg 3 ends ~128 deg; the
    // corner loop is designed to end at 175 deg (facing ~west, turret-comfortable).
    private static final double SHOOT_HEADING_ROW_DEG = 128;
    private static final double SHOOT_HEADING_CORNER_DEG = 175;

    /** BLUE uses the authored coordinates; RED mirrors them across the field. */
    protected abstract Alliance alliance();

    private boolean mirror;

    private PathChain rowCollect;   // start -> row start -> sweep end (approach+sweep, continuous)
    private PathChain rowReturn;    // sweep end -> shoot (reversed tangent), after the sweep-end wait
    private PathChain cornerOut;    // shoot -> top -> second point (the intake sweep)
    private PathChain cornerReturn; // second point -> shoot (the return, after the dwell)

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
        Constants.turretAimOffsetDegrees = turretOffset(FIRST_SHOT_TURRET_OFFSET_DEGREES)
                + (mirror ? RED_FIRST_SHOT_EXTRA_DEGREES : 0.0);
        // Auto shoot: suppress ball detection until each ~480-deg volley rotation finishes.
        robot.spindexer.setAutoMode(true);

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
                // Preload fired: switch to the rest-shot turret bias for every other shot.
                .then(instant(() -> Constants.turretAimOffsetDegrees =
                        turretOffset(REST_SHOT_TURRET_OFFSET_DEGREES)
                                + (mirror ? RED_REST_SHOT_EXTRA_DEGREES : 0.0)))
                // ----- last row (segs 1-3, one continuous drive) -----
                .then(instant(() -> aimTargetPose = shootPoseRow()))
                // Intake on before the run so it is fully spun up entering the row.
                .then(intakeOn())
                // Collect: approach + sweep in one continuous drive; slowZoneControl
                // throttles to half power in the row band. No stop between approach and
                // sweep. Restore full power after.
                .then(deadline(
                        followWithTimeout(rowCollect, DRIVE_TIMEOUT_MS, FULL_POWER),
                        slowZoneControl()))
                .then(instant(() -> robot.drivetrain.follower.setMaxPower(FULL_POWER)))
                // Pause at the end of the sweep so the swept artifacts seat.
                .then(waitMs(ROW_SWEEP_END_WAIT_MS))
                // Back out to the shoot point, then eject a possible 4th and fire.
                .then(followWithTimeout(rowReturn, DRIVE_TIMEOUT_MS, FULL_POWER))
                .then(ejectThenShoot());

        // ----- corner stack, CORNER_CYCLES times, all from the same shoot point -----
        // Pre-aim at the corner return heading for the rest of the run.
        auto = auto.then(instant(() -> aimTargetPose = shootPoseCorner()));
        for (int i = 0; i < CORNER_CYCLES; i++) {
            auto = auto
                    // rotateShootCW() cleared intaking on the last shot, so re-arm it.
                    .then(intakeOn())
                    // Intake pass into the stack; cornerSlowControl drops to the corner
                    // feed power while heading INWARD through the stack band. Full after.
                    .then(deadline(
                            followWithTimeout(cornerOut, DRIVE_TIMEOUT_MS, FULL_POWER),
                            cornerSlowControl()))
                    .then(instant(() -> robot.drivetrain.follower.setMaxPower(FULL_POWER)))
                    // DWELL at the end of the intake (still intaking) so the collected
                    // balls seat - this is the only dwell in the cycle; there is none
                    // after firing.
                    .then(waitMs(PICKUP_SEAT_MS))
                    .then(conditional(() -> robot.spindexer.getBallCount() >= 3,
                            intakeOff(), noOp()))
                    // Return to the shoot point and fire; the shot moves straight into
                    // the next cycle with no post-fire wait.
                    .then(followWithTimeout(cornerReturn, DRIVE_TIMEOUT_MS, FULL_POWER))
                    .then(ejectThenShoot());
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
        if (current != null) {
            // Turret: normal continuous auto-aim - always track the goal from the live pose
            // (nothing special). Gated on autoAimEnabled so the end-of-auto turret home isn't
            // overridden.
            if (robot.turret.autoAimEnabled) {
                TractorBeam.aimTurret(current, robot, Alliance.current);
            }
            // Flywheel: live far interpolation while collecting, but latch to the
            // interpolated shoot-pose value on the final run-in so it is spun up on
            // arrival instead of chasing it. Same interpolation value the shot uses -
            // shot speed unchanged, only the approach spins up eagerly.
            if (aimTargetPose != null) {
                double dist = Math.hypot(aimTargetPose.getX() - current.getX(),
                        aimTargetPose.getY() - current.getY());
                if (dist <= APPROACH_SPINUP_DISTANCE) {
                    Pose turretPose = TurretLocation.getTurretPose(aimTargetPose);
                    robot.shooter.setTarget(WaveLength.getFarVelocityWithInterpolation(turretPose, Alliance.current));
                    robot.shooter.setHoodPosition(WaveLength.getFarHoodWithInterpolation(turretPose, Alliance.current));
                } else {
                    robot.shooter.useFarInterpolation();
                }
            }
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
    /**
     * At a shoot pose (end of a return path): stop indexing, reverse the intake to spit
     * out any 4th artifact wedged at the intake, then fire. This guarantees we never shoot
     * with 4 loaded. The eject window runs IN PARALLEL with the shot-readiness settle
     * (flywheel/pose/heading/turret) rather than before it, so we wait max(eject, settle)
     * instead of eject + settle - the settle is usually already done on arrival, so the
     * eject window is the only real wait.
     */
    private Command ejectThenShoot() {
        return robot.spindexer.setIntaking(false)
                .then(robot.intake.reverse())
                .then(parallel(
                        waitMs(EJECT_REVERSE_MS),
                        waitUntil(this::shotReady).raceWith(waitMs(SHOOT_READY_TIMEOUT_MS))))
                .then(robot.intake.off())
                .then(fireVolley());
    }

    /**
     * Ready to fire: flywheel up to speed, robot on the shoot pose, CHASSIS settled onto
     * the shoot heading (so the fixed turret aim is correct), and the turret at target.
     */
    private boolean shotReady() {
        return robot.shooter.atTarget()
                && nearShootPose()
                && chassisSettled()
                && robot.turret.atTarget(TURRET_AIM_TOLERANCE_DEG);
    }

    /**
     * Rotate the spindexer to fire the whole volley, then wait for it to finish - i.e. all
     * balls are out - before moving on, so the robot never drives off mid-volley. The wait
     * is timeout-capped so a stuck spindexer can't stall the routine.
     */
    private Command fireVolley() {
        return robot.spindexer.rotateShootCW()
                .then(waitUntil(() -> !robot.spindexer.isMoving())
                        .raceWith(waitMs(SHOOT_FIRE_TIMEOUT_MS)));
    }

    /**
     * Wait until the shot is lined up (or the ready timeout elapses), then fire. Used for
     * the preload, which has no eject step.
     */
    private Command shootWhenReady() {
        return shootWhenReady(SHOOT_READY_TIMEOUT_MS);
    }

    private Command shootWhenReady(long readyTimeoutMs) {
        return waitUntil(this::shotReady)
                .raceWith(waitMs(readyTimeoutMs))
                .then(fireVolley());
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
     * Corner throttle: CORNER_INTAKE_POWER only while the robot is driving INWARD (authored
     * x decreasing, toward x=0) through the stack band [CORNER_SLOW_MIN_X, MAX_X]; full
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
                    power = CORNER_INTAKE_POWER;
                }
                lastAuthoredX = authoredX;
            }
            robot.drivetrain.follower.setMaxPower(power);
        });
    }

    /** Aim bias: authored for BLUE, negated for RED so the physical correction matches. */
    private double turretOffset(double blueOffsetDegrees) {
        return mirror ? -blueOffsetDegrees : blueOffsetDegrees;
    }

    /** True once the robot is within SHOOT_POSE_TOLERANCE_IN of the current shoot pose. */
    private boolean nearShootPose() {
        Pose p = robot.drivetrain.getPose();
        if (p == null || aimTargetPose == null) return false;
        return Math.hypot(p.getX() - aimTargetPose.getX(), p.getY() - aimTargetPose.getY())
                <= SHOOT_POSE_TOLERANCE_IN;
    }

    /**
     * True once the chassis heading has settled onto the shoot pose heading. The turret
     * aims at the FIXED pose (which assumes the chassis is at that heading), so we hold
     * the shot until the chassis has actually finished turning onto it - otherwise the
     * fixed aim would be off by however far the chassis still had to turn.
     */
    private boolean chassisSettled() {
        Pose p = robot.drivetrain.getPose();
        if (p == null || aimTargetPose == null) return false;
        double errDeg = ((Math.toDegrees(p.getHeading() - aimTargetPose.getHeading()) + 540) % 360) - 180;
        return Math.abs(errDeg) <= HEADING_SETTLE_TOLERANCE_DEG;
    }

    // ----- geometry ---------------------------------------------------------

    /** Far shoot point the preload is fired from, and where the robot starts. */
    private Pose startPose() {
        // RED uses an explicit field start pose (not the mirrored BLUE start).
        if (mirror) {
            return new Pose(79.44, 8.13, Math.toRadians(90));
        }
        return pose(63.74, 8.235, START_HEADING_DEG);
    }

    /** Shoot point after the ROW return (seg 3's reversed-tangent arrival heading). */
    private Pose shootPoseRow() {
        return pose(58, 11, SHOOT_HEADING_ROW_DEG);
    }

    /** Shoot point after a CORNER return (reversed-tangent arrival heading). */
    private Pose shootPoseCorner() {
        return pose(58, 11, SHOOT_HEADING_CORNER_DEG);
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
        // Collection: approach curve + straight sweep, one CONTINUOUS chain (no stop
        // between them). The routine then pauses ROW_SWEEP_END_WAIT_MS at the sweep end
        // before following rowReturn back to the shoot point.
        rowCollect = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(point(63.74, 8.235), point(63.74, 35.5), point(40.5, 35.5)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(point(40.5, 35.5), point(11.5, 35.5)))
                .setTangentHeadingInterpolation()
                .build();
        rowReturn = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(point(11.5, 35.5), point(35, 35.5), point(58, 11)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // Human-player corner cycle, split into the intake pass (cornerOut) and the
        // return (cornerReturn) so the robot can DWELL at the end of the intake - in the
        // corner, where it just collected - instead of after the shot. cornerOut runs
        // straight up-left to (12,22), then straight down to (12,11) holding a CONSTANT
        // -130 deg heading. cornerReturn is the reversed-tangent back to the shoot point
        // (52.5,11), arriving ~west so the turret stays in range. Both re-followed each cycle.
        //   out  path 4  line   shoot -> (13,33)        tangential
        //   out  path 5  line   (13,33) -> (13,12)      constant -130
        //   return path 6  curve  (13,12) -> shoot        tangential reversed (arrives ~174)
        cornerOut = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(point(58, 11), cornerPoint(16, 20)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(cornerPoint(16, 20), cornerPoint(16, 15)))
                .setConstantHeadingInterpolation(hdg(-120))
                .build();
        cornerReturn = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(cornerPoint(16, 15), cornerPoint(33, 15), point(58, 11)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    /** A geometric point (heading irrelevant to a Bezier); x mirrored for RED. */
    private Pose point(double x, double y) {
        return new Pose(mirror ? FIELD_WIDTH - x : x, y);
    }

    /** Corner-cycle point: like point(), plus a RED-only +RED_CORNER_X_SHIFT field-x shift. */
    private Pose cornerPoint(double x, double y) {
        return new Pose(mirror ? FIELD_WIDTH - x + RED_CORNER_X_SHIFT : x, y);
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

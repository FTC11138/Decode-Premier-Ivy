package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.math.TractorBeam;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.util.Constants;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;

/**
 * "Close, no-gate" fallback autonomous, ported from the old FTCLib /
 * CommandScheduler "Auto_12" into this repo's Ivy + Pedro command framework.
 *
 * Sequence (preload + 3 collected volleys, all fired from one shoot pose):
 *   0. Drive start -> shoot pose while the flywheel spins up, fire the preload.
 *   1. Intake row 1: out to (15, 88), return to the shoot pose, fire.
 *   2. Intake row 2: out to (7, 64) with a curved return, fire.
 *   3. Intake row 3: out to (7, 42), return, fire.
 *   4. Stop the shooter + intake and park at (36, 72).
 *
 * The turret auto-aims at the goal every loop; flywheel velocity + hood are
 * interpolated from pose (close table). Coordinates are authored for BLUE; RED
 * mirrors them across the field (FIELD_WIDTH) with pose()/hdg(), the same
 * convention used everywhere else in this codebase. Every drive is wrapped in a
 * timeout so a stuck follower can never stall the routine.
 *
 * Shot timing uses the original auto's fixed waits (no atTarget() gating) so the
 * behavior matches what was tuned before; swap shootWhenReady()-style gating in
 * later if you want it to wait on the flywheel instead.
 */
public abstract class AutoCloseNoGateFallBack extends RobotOpMode {
    private static final double FIELD_WIDTH = 141.5;

    // Run above the default priority (0) so the spindexer/intake periodics can't
    // override and kill this long Sequential mid-run (see GateIntakeWithFarAuto).
    private static final int AUTO_PRIORITY = 1;

    // Generous per-drive safety timeout: only fires if a follower gets stuck, so a
    // path that never settles can't hang the auto. Normal drives finish well under it.
    private static final long DRIVE_TIMEOUT_MS = 5000;

    // Fixed shot timings carried over from the original auto.
    private static final long PRELOAD_CHARGE_MS = 1700; // let the flywheel spin up
    private static final long SHOT_CHARGE_MS = 1000;    // re-charge before each later shot
    private static final long AFTER_SHOT_MS = 300;
    private static final long AFTER_PRELOAD_INTAKE_MS = 500;
    private static final long BEFORE_SHOOT_MS = 200;

    // Reduced drive power on the deep-reach intake legs so the robot doesn't blow
    // past the balls it is collecting (matches the old PathCommand power args).
    private static final double ROW1_INTAKE_POWER = 0.65;
    private static final double ROW23_INTAKE_POWER = 0.6;

    /** BLUE uses the authored coordinates; RED mirrors them across the field. */
    protected abstract Alliance alliance();

    private boolean mirror;

    private PathChain shoot0Path;
    private PathChain intake11Path;
    private PathChain intake12Path;
    private PathChain shoot1Path;
    private PathChain intake21Path;
    private PathChain intake22Path;
    private PathChain shoot2Path;
    private PathChain intake31Path;
    private PathChain intake32Path;
    private PathChain shoot3Path;
    private PathChain movePath;

    @Override
    public void init() {
        super.init();

        mirror = alliance() == Alliance.RED;
        Alliance.current = alliance();

        robot.drivetrain.setStartingPose(pose(30, 134.5, 90));
        robot.turret.setStartingAngle(Constants.turretHomedAngleDegrees);
        robot.turret.enableAutoAim();

        buildPaths();
    }

    @Override
    public void start() {
        robot.spindexer.resetEncoderZero();
        // Close-range shots: interpolate flywheel velocity + hood from pose.
        robot.shooter.useCloseInterpolation();
        robot.shooter.turnOn();

        followWithTimeout(shoot0Path, DRIVE_TIMEOUT_MS)
                .then(waitMs(PRELOAD_CHARGE_MS))
                .then(robot.spindexer.rotateShootCW())
                // Preload away: start intaking and collect row 1.
                .then(intakeOn())
                .then(waitMs(AFTER_PRELOAD_INTAKE_MS))
                .then(followWithTimeout(intake11Path, DRIVE_TIMEOUT_MS))
                .then(followWithTimeout(intake12Path, DRIVE_TIMEOUT_MS, ROW1_INTAKE_POWER))
                .then(waitMs(BEFORE_SHOOT_MS))
                .then(followWithTimeout(shoot1Path, DRIVE_TIMEOUT_MS))
                .then(waitMs(SHOT_CHARGE_MS))
                .then(robot.spindexer.rotateShootCW())
                .then(waitMs(AFTER_SHOT_MS))
                // Row 2.
                .then(intakeOn())
                .then(followWithTimeout(intake21Path, DRIVE_TIMEOUT_MS))
                .then(followWithTimeout(intake22Path, DRIVE_TIMEOUT_MS, ROW23_INTAKE_POWER))
                .then(waitMs(BEFORE_SHOOT_MS))
                .then(followWithTimeout(shoot2Path, DRIVE_TIMEOUT_MS))
                .then(waitMs(SHOT_CHARGE_MS))
                .then(robot.spindexer.rotateShootCW())
                .then(waitMs(AFTER_SHOT_MS))
                // Row 3.
                .then(intakeOn())
                .then(followWithTimeout(intake31Path, DRIVE_TIMEOUT_MS))
                .then(followWithTimeout(intake32Path, DRIVE_TIMEOUT_MS, ROW23_INTAKE_POWER))
                .then(waitMs(BEFORE_SHOOT_MS))
                .then(followWithTimeout(shoot3Path, DRIVE_TIMEOUT_MS))
                .then(waitMs(SHOT_CHARGE_MS))
                .then(robot.spindexer.rotateShootCW())
                .then(waitMs(AFTER_SHOT_MS))
                // Done: stop shooter + intake, then park.
                .then(instant(() -> robot.shooter.turnOff()))
                .then(intakeOff())
                .then(followWithTimeout(movePath, DRIVE_TIMEOUT_MS))
                .setPriority(AUTO_PRIORITY)
                .schedule();
    }

    @Override
    public void loop() {
        Pose current = robot.drivetrain.getPose();
        if (robot.turret.autoAimEnabled && current != null) {
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

    /** Follow a path but give up after timeoutMs so a stuck follower can't stall the auto. */
    private Command followWithTimeout(PathChain path, long timeoutMs) {
        return robot.drivetrain.followPath(path).raceWith(waitMs(timeoutMs));
    }

    private Command followWithTimeout(PathChain path, long timeoutMs, double maxPower) {
        return robot.drivetrain.followPath(path, maxPower).raceWith(waitMs(timeoutMs));
    }

    // ----- paths ------------------------------------------------------------

    private void buildPaths() {
        Pose start = pose(30, 134.5, 90);
        Pose shoot = pose(50, 103, 139);
        Pose intake11 = pose(54, 88, 180);       // old: 144 - 90
        Pose intake12 = pose(15, 88, 180);
        Pose intake21 = pose(47.5, 64, 180);
        Pose intake22 = pose(7, 64, 180);
        Pose intake31 = pose(47, 42, 180);
        Pose intake32 = pose(7, 42, 180);
        Pose park = pose(36, 72, 180);           // old: 144 - 108
        Pose shoot2Control = point(92.5, 55.7);  // old: 144 - 51.5

        shoot0Path = line(start, shoot);
        intake11Path = line(shoot, intake11);
        intake12Path = line(intake11, intake12);
        shoot1Path = line(intake12, shoot);
        intake21Path = line(shoot, intake21);
        intake22Path = line(intake21, intake22);
        shoot2Path = curve(intake22, shoot2Control, shoot);
        intake31Path = line(shoot, intake31);
        intake32Path = line(intake31, intake32);
        shoot3Path = line(intake32, shoot);
        movePath = line(shoot, park);
    }

    private PathChain line(Pose from, Pose to) {
        return robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(from, to))
                .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
                .build();
    }

    private PathChain curve(Pose from, Pose control, Pose to) {
        return robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(from, control, to))
                .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
                .build();
    }

    /** BLUE-authored pose; mirrored across FIELD_WIDTH (x and heading) for RED. */
    private Pose pose(double x, double y, double headingDeg) {
        return new Pose(mirror ? FIELD_WIDTH - x : x, y, hdg(headingDeg));
    }

    /** A control point (no heading); x mirrored for RED. */
    private Pose point(double x, double y) {
        return new Pose(mirror ? FIELD_WIDTH - x : x, y);
    }

    private double hdg(double headingDeg) {
        double radians = Math.toRadians(headingDeg);
        return mirror ? Math.PI - radians : radians;
    }
}

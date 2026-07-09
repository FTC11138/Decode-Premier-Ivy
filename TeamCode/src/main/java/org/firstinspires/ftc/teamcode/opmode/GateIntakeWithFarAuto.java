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
import static com.pedropathing.ivy.commands.Commands.waitUntil;

/**
 * Shared logic for the "GateIntakeWithFar" autonomous.
 *
 * Sequence:
 *   1. Drive (26.3,128.8) -> (57,82) and fire the preload when the shooter is ready.
 *   2. Second row: sweep-intake the curve out to (12,58) and back, then shoot.
 *   3. Gate x3: drive (57,82) -> (14,59) -> (12,56), intake until 3 balls (or a
 *      3 s timeout), drive back to (57,82), and shoot.
 *   4. First row: sweep-intake the line out to (11,82) and back, then shoot.
 *
 * The turret auto-aims the whole time and the flywheel stays spun up (interpolated
 * by position). Robot pose, turret angle, and Alliance.current all carry into
 * TeleOp automatically.
 *
 * Coordinates are authored for BLUE (left side); RED mirrors them across the field.
 */
public abstract class GateIntakeWithFarAuto extends RobotOpMode {
    private static final double FIELD_WIDTH = 141.5;
    private static final long GATE_INTAKE_TIMEOUT_MS = 3000;
    private static final int GATE_CYCLES = 3;
    private static final long SHOOT_SETTLE_MS = 400;

    /** BLUE uses the authored coordinates; RED mirrors them across the field. */
    protected abstract Alliance alliance();

    private boolean mirror;
    private long gateIntakeStartMs;

    private PathChain startToShoot;
    private PathChain secondRowSweep;
    private PathChain shootToGate;
    private PathChain gateToShoot;
    private PathChain firstRowSweep;

    @Override
    public void init() {
        super.init();

        mirror = alliance() == Alliance.RED;
        Alliance.current = alliance();

        robot.drivetrain.setStartingPose(pose(26.3, 128.8, 136.8));
        robot.turret.setStartingAngle(Constants.turretHomedAngleDegrees);
        robot.turret.enableAutoAim();

        buildPaths();
    }

    @Override
    public void start() {
        robot.spindexer.resetEncoderZero();
        robot.shooter.useInterpolation();
        robot.shooter.turnOn();

        Command auto = robot.drivetrain.followPath(startToShoot)
                .then(shootWhenReady())
                .then(sweepIntake(secondRowSweep)).then(shootWhenReady());
        for (int i = 0; i < GATE_CYCLES; i++) {
            auto = auto.then(gateCycle());
        }
        auto = auto.then(sweepIntake(firstRowSweep)).then(shootWhenReady());
        auto.schedule();
    }

    @Override
    public void loop() {
        if (robot.turret.autoAimEnabled) {
            TractorBeam.aimTurret(robot.drivetrain.getPose(), robot, Alliance.current);
        }
        super.loop();
    }
    private Command gateCycle() {
        return robot.intake.on()
                .then(robot.spindexer.setIntaking(true))
                .then(robot.drivetrain.followPath(shootToGate))
                .then(instant(() -> gateIntakeStartMs = System.currentTimeMillis()))
                .then(waitUntil(() -> robot.spindexer.getBallCount() >= 3
                        || System.currentTimeMillis() - gateIntakeStartMs >= GATE_INTAKE_TIMEOUT_MS))
                .then(robot.intake.off())
                .then(robot.spindexer.setIntaking(false))
                .then(robot.drivetrain.followPath(gateToShoot))
                .then(shootWhenReady());
    }
    private Command sweepIntake(PathChain path) {
        return robot.intake.on()
                .then(robot.spindexer.setIntaking(true))
                .then(robot.drivetrain.followPath(path))
                .then(robot.intake.off())
                .then(robot.spindexer.setIntaking(false));
    }

    /** Wait until the shooter is genuinely at target, then fire everything. */
    private Command shootWhenReady() {
        return waitUntil(() -> robot.shooter.atTarget())
                .then(robot.spindexer.rotateShootCW())
                .then(waitUntil(() -> !robot.spindexer.isMoving()))
                .then(waitMs(SHOOT_SETTLE_MS));
    }

    // ----- Paths ------------------------------------------------------------

    private void buildPaths() {
        startToShoot = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(pose(26.3, 128.8, 136.8), pose(57, 82, 180)))
                .setLinearHeadingInterpolation(hdg(136.8), hdg(180))
                .build();
        secondRowSweep = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierCurve(pose(57, 82, 180), pose(60.5, 58, 180), pose(12, 58, 180)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(pose(12, 58, 180), pose(60.5, 58, 180), pose(57, 82, 180)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        shootToGate = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(pose(57, 82, 180), pose(14, 59, 160)))
                .setLinearHeadingInterpolation(hdg(180), hdg(160))
                .addPath(new BezierLine(pose(14, 59, 160), pose(12, 56, 140)))
                .setLinearHeadingInterpolation(hdg(160), hdg(140))
                .build();
        gateToShoot = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(pose(12, 56, 140), pose(57, 82, 180)))
                .setLinearHeadingInterpolation(hdg(140), hdg(180))
                .build();
        firstRowSweep = robot.drivetrain.follower.pathBuilder()
                .addPath(new BezierLine(pose(57, 82, 180), pose(11, 82, 180)))
                .setLinearHeadingInterpolation(hdg(180), hdg(180))
                .addPath(new BezierLine(pose(11, 82, 180), pose(57, 82, 180)))
                .setLinearHeadingInterpolation(hdg(180), hdg(180))
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

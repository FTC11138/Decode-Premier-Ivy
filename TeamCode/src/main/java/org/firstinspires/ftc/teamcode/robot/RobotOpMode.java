package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.util.PanelsFieldDrawing;

import java.util.List;

import static com.pedropathing.ivy.Scheduler.schedule;

public abstract class RobotOpMode extends OpMode {
    protected Robot robot;

    // Cached hub handles so every loop pays one bulk-read USB transaction per hub
    // instead of a separate round-trip per encoder/velocity/digital-input read.
    private List<LynxModule> hubs;

    // Loop-time instrumentation so the bulk-caching win is measurable on the
    // driver station / dashboard. Exponential moving average smooths the jitter.
    private long lastLoopNanos = 0;
    private double loopMsAverage = 0;

    @Override
    public void init() {
        Scheduler.reset();
        robot = new Robot(this);
        PanelsFieldDrawing.init();

        // MANUAL bulk caching: we clear the cache once at the top of every loop
        // (below) and every hardware read afterwards is served from that single
        // snapshot. Note: motor getCurrent() is NOT part of the bulk packet and
        // still costs its own transaction, so keep current reads to one per loop.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (int i = 0; i < hubs.size(); i++) {
            hubs.get(i).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        schedule(
                robot.drivetrain.periodic(),
                robot.shooter.periodic(),
                robot.turret.periodic(),
                robot.spindexer.periodic(),
                robot.intake.periodic(),
                robot.leds.periodic()
        );
    }

    @Override
    public void init_loop() {
        clearBulkCaches();
        Scheduler.execute();
        addRobotPoseTelemetry();
        PanelsFieldDrawing.drawRobot(robot.drivetrain.getPose());
        robot.telemetry.update();
    }

    @Override
    public void loop() {
        clearBulkCaches();
        Scheduler.execute();
        addRobotPoseTelemetry();
        addLoopTimeTelemetry();
        PanelsFieldDrawing.drawRobot(robot.drivetrain.getPose());
        robot.telemetry.update();
    }

    private void addLoopTimeTelemetry() {
        long now = System.nanoTime();
        if (lastLoopNanos != 0) {
            double loopMs = (now - lastLoopNanos) / 1e6;
            // 0.1 smoothing weight on the newest sample.
            loopMsAverage = loopMsAverage == 0 ? loopMs : loopMsAverage * 0.9 + loopMs * 0.1;
            robot.telemetry.addData("Loop ms", "%.2f (avg %.2f, %.0f Hz)",
                    loopMs, loopMsAverage, 1000.0 / Math.max(loopMsAverage, 1e-3));
        }
        lastLoopNanos = now;
    }

    // Invalidate the cached bulk snapshot exactly once per loop so the first read
    // of the loop fetches fresh data and every read after it reuses that snapshot.
    private void clearBulkCaches() {
        if (hubs == null) {
            return;
        }
        for (int i = 0; i < hubs.size(); i++) {
            hubs.get(i).clearBulkCache();
        }
    }

    private void addRobotPoseTelemetry() {
        Pose pose = robot.drivetrain.getPose();
        if (pose == null) {
            robot.telemetry.addData("Robot Pose", "Unavailable");
            return;
        }

        robot.telemetry.addData("Robot X", pose.getX());
        robot.telemetry.addData("Robot Y", pose.getY());
        robot.telemetry.addData("Robot Heading Rad", pose.getHeading());
        robot.telemetry.addData("Robot Heading Deg", Math.toDegrees(pose.getHeading()));
        robot.telemetry.addData(
                "Robot Pose",
                "x=%.2f, y=%.2f, h=%.1f deg",
                pose.getX(),
                pose.getY(),
                Math.toDegrees(pose.getHeading())
        );
    }
}

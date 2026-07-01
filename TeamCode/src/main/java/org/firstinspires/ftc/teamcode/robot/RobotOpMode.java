package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.util.PanelsFieldDrawing;

import static com.pedropathing.ivy.Scheduler.schedule;

public abstract class RobotOpMode extends OpMode {
    protected Robot robot;

    @Override
    public void init() {
        Scheduler.reset();
        robot = new Robot(this);
        PanelsFieldDrawing.init();

        schedule(
                robot.drivetrain.periodic(),
                robot.shooter.periodic(),
                robot.turret.periodic(),
                robot.spindexer.periodic(),
                robot.intake.periodic()
        );
    }

    @Override
    public void init_loop() {
        Scheduler.execute();
        addRobotPoseTelemetry();
        PanelsFieldDrawing.drawRobot(robot.drivetrain.getPose());
        robot.telemetry.update();
    }

    @Override
    public void loop() {
        Scheduler.execute();
        addRobotPoseTelemetry();
        PanelsFieldDrawing.drawRobot(robot.drivetrain.getPose());
        robot.telemetry.update();
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

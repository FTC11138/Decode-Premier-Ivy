package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.math.TractorBeam;
import org.firstinspires.ftc.teamcode.math.TurretLocation;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "TeleOp", group = "Competition")
@Config
public class TeleOp_Solo extends RobotOpMode {
    private boolean gamepad1TouchpadWasDown = false;

    @Override
    public void init() {
        super.init();

        robot.drivetrain.usePreviousStartingPose();
        robot.drivetrain.setFieldCentricEnabled(true);
        robot.drivetrain.clearFieldCentricHeadingReset();
        robot.turret.usePreviousStartingAngle();
    }

    @Override
    public void start() {
        robot.intake.on().schedule();
    }

    @Override
    public void loop() {
        if (gamepad1.touchpad && !gamepad1TouchpadWasDown) {
            robot.drivetrain.resetFieldCentricHeading(Alliance.current);
        }
        gamepad1TouchpadWasDown = gamepad1.touchpad;

        if (Math.abs(gamepad1.right_stick_x) >= 0.1) robot.drivetrain.unlockHeading();

        robot.drivetrain.fieldCentricDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                Alliance.current
        );

        Pose turretPose = TurretLocation.getTurretPose(robot.drivetrain.getPose());

        TractorBeam.aimTurret(robot.drivetrain.getPose(), robot, Alliance.current);

        if (robot.drivetrain.getPose().getY() > 48) {
            Intake.slowPower = -1;
        } else {
            Intake.slowPower = -0.6;
        }

        if (gamepad1.rightTriggerWasPressed()) {
            robot.intake.slowDown();
        }

        if (gamepad1.rightTriggerWasReleased()) {
            robot.intake.speedUp();
        }

//        if (gamepad1.rightBumperWasPressed()) robot.intake.toggle().schedule();
        if (gamepad1.rightBumperWasPressed()) robot.intake.off().schedule();
        if (gamepad1.rightBumperWasReleased()) robot.intake.on().schedule();
        if (gamepad1.leftBumperWasPressed()) robot.intake.shortReverse().schedule();

        if (gamepad1.triangleWasPressed()) robot.shooter.toggle();

        if (gamepad1.squareWasPressed()) robot.drivetrain.gateHeading(Alliance.current);


//        if (gamepad2.crossWasPressed()) robot.turret.home();
        if (gamepad2.triangleWasPressed()) robot.turret.forceOff = !robot.turret.forceOff;

        if (gamepad2.leftBumperWasPressed()) Alliance.current = Alliance.RED;
        if (gamepad2.rightBumperWasPressed()) Alliance.current = Alliance.BLUE;

        if (gamepad2.crossWasPressed()) robot.drivetrain.setPose(
                Alliance.current == Alliance.RED ? new Pose(8.1, 7.5, 0) : new Pose(141.5 - 8.1, 7.5, Math.PI)
        );

        if (gamepad2.circleWasPressed()) robot.drivetrain.setPose(robot.drivetrain.getPose().withHeading(
                Alliance.current == Alliance.RED ? 0 : Math.PI
        ));


        if (gamepad2.dpadLeftWasPressed()) robot.turret.moveLeft();
        if (gamepad2.dpadRightWasPressed()) robot.turret.moveRight();

        robot.telemetry.addData("Turret X", turretPose.getX());
        robot.telemetry.addData("Turret Y", turretPose.getY());
        robot.telemetry.addData("Alliance", Alliance.current);

        super.loop();
    }
}

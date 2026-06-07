package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.LLPoseResetter;
import org.firstinspires.ftc.teamcode.math.TractorBeam;
import org.firstinspires.ftc.teamcode.math.TurretLocation;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.util.Constants;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

@TeleOp(name = "TeleOp", group = "Competition")
public class TeleOp_Solo extends RobotOpMode {
    private boolean teleOpEnabled = false;
    private boolean gamepad1StartWasDown = false;
    private boolean gamepad1TouchpadWasDown = false;
    private boolean gamepad1LeftTriggerWasDown = false;
    private boolean gamepad1RightTriggerWasDown = false;
    private boolean gamepad1DpadUpWasDown = false;
    private boolean gamepad1DpadDownWasDown = false;
    private boolean gamepad1DpadLeftWasDown = false;
    private boolean gamepad1DpadRightWasDown = false;
    private boolean intakeEnabled = false;

    Pose turretPose;
    private LLPoseResetter llPoseResetter;

    @Override
    public void init() {
        super.init();

        llPoseResetter = new LLPoseResetter(hardwareMap);
        robot.drivetrain.usePreviousStartingPose();
        robot.drivetrain.startTeleOpDrive();
        robot.drivetrain.setFieldCentricEnabled(true);
        robot.drivetrain.clearFieldCentricHeadingReset();
        robot.turret.usePreviousStartingAngle();
        robot.intake.off().schedule();
        robot.spindexer.setIntaking(false).schedule();
        intakeEnabled = false;
        setAllianceLed();
    }

    @Override
    public void start() {
        teleOpEnabled = false;
        llPoseResetter.start();
    }

    @Override
    public void stop() {
        llPoseResetter.stop();
    }

    @Override
    public void loop() {
        boolean start = gamepad1.start;
        if (start && !gamepad1StartWasDown) {
            teleOpEnabled = true;
            gamepad1.rumble(2000);
        }
        gamepad1StartWasDown = start;

        if (gamepad1.touchpad && !gamepad1TouchpadWasDown) {
            robot.drivetrain.resetFieldCentricHeading(Alliance.current);
            gamepad1.rumble(500);
            gamepad1.setLedColor(0, 1, 0, 1000);
        }
        gamepad1TouchpadWasDown = gamepad1.touchpad;

        if (teleOpEnabled) {
            runDriverControls();
        }

        turretPose = TurretLocation.getTurretPose(robot.drivetrain.getPose());

        robot.telemetry.addData("TeleOp Enabled", teleOpEnabled);
        robot.telemetry.addData("Turret X", turretPose.getX());
        robot.telemetry.addData("Turret Y", turretPose.getY());
        robot.telemetry.addData("Alliance", Alliance.current);
        robot.telemetry.addData("LL Pose Reset", llPoseResetter.getStatus());
        robot.telemetry.addData("Spindexer Ball Count", robot.spindexer.getBallCount());

        super.loop();
    }

    private void runDriverControls() {
        if (Math.abs(gamepad1.right_stick_x) >= Constants.driveTurnDeadband) {
            robot.drivetrain.unlockHeading();
        }

        robot.drivetrain.fieldCentricDrive(
                -gamepad1.left_stick_y * Constants.driveForwardMultiplier,
                -gamepad1.left_stick_x * Constants.driveStrafeMultiplier,
                -gamepad1.right_stick_x * Constants.driveTurnMultiplier,
                Alliance.current
        );

        TractorBeam.aimTurret(robot.drivetrain.getPose(), robot, Alliance.current);

//        if (robot.drivetrain.getPose().getY() > Constants.intakeSlowPowerYThreshold) {
//            robot.intake.speedUp();
//        } else {
//            robot.intake.slowDown();
//        }

        boolean leftTrigger = gamepad1.left_trigger > 0.25;
        if (leftTrigger && !gamepad1LeftTriggerWasDown) {
            intakeEnabled = !intakeEnabled;
            if (intakeEnabled) {
                robot.intake.on().then(robot.spindexer.setIntaking(true)).schedule();
            } else {
                robot.intake.off().then(robot.spindexer.setIntaking(false)).schedule();
            }
        }
        gamepad1LeftTriggerWasDown = leftTrigger;

        boolean rightTrigger = gamepad1.right_trigger > 0.25;
        if (rightTrigger && !gamepad1RightTriggerWasDown) {
            conditional(
                    () -> robot.shooter.isOn() && robot.shooter.atTarget(),
                    robot.spindexer.rotateShootCW(),
                    robot.spindexer.stop()
            ).then(waitUntil(() -> !robot.spindexer.isMoving()))
                    .then(robot.intake.on(), robot.spindexer.setIntaking(true)).schedule();
            intakeEnabled = true;
        }
        gamepad1RightTriggerWasDown = rightTrigger;

        if (gamepad1.leftBumperWasPressed()) {
            robot.intake.shortReverse().schedule();
            intakeEnabled = true;
        }

        if (gamepad1.rightStickButtonWasPressed()) {
            if (llPoseResetter.resetPose(robot.drivetrain)) {
                gamepad1.rumble(500);
            }
        }
        if (gamepad1.rightBumperWasPressed()) {
            robot.spindexer.rotateShootCW()
                    .then(waitUntil(() -> !robot.spindexer.isMoving()))
                    .then(robot.intake.on(), robot.spindexer.setIntaking(true))
                    .schedule();
            intakeEnabled = true;
        }

        if (gamepad1.crossWasPressed()) {
            instant(() -> {
                robot.shooter.setTarget(Constants.shootVelFar);
                robot.shooter.setHoodPosition(Constants.shootHoodFar);
                robot.shooter.turnOn();
            }).schedule();
        }

        if (gamepad1.circleWasPressed()) {
            instant(() -> {
                robot.shooter.useInterpolation();
                robot.shooter.turnOn();
            }).schedule();
        }

        if (gamepad1.triangleWasPressed()) {
            robot.shooter.toggle();
        }

        if (gamepad1.squareWasPressed()) {
            robot.drivetrain.gateHeading(Alliance.current);
        }

        if (gamepad1.dpad_up && !gamepad1DpadUpWasDown) {
            robot.spindexer.rotate15CW().schedule();
        }
        gamepad1DpadUpWasDown = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !gamepad1DpadDownWasDown) {
            instant(robot.shooter::turnOff).schedule();
        }
        gamepad1DpadDownWasDown = gamepad1.dpad_down;

        if (gamepad1.dpad_left && !gamepad1DpadLeftWasDown) {
            robot.turret.moveLeft();
        }
        gamepad1DpadLeftWasDown = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !gamepad1DpadRightWasDown) {
            robot.turret.moveRight();
        }
        gamepad1DpadRightWasDown = gamepad1.dpad_right;

        if (gamepad2.triangleWasPressed()) {
            robot.turret.forceOff = !robot.turret.forceOff;
        }

        if (gamepad2.leftBumperWasPressed()) {
            Alliance.current = Alliance.RED;
            setAllianceLed();
        }

        if (gamepad2.rightBumperWasPressed()) {
            Alliance.current = Alliance.BLUE;
            setAllianceLed();
        }

        if (gamepad2.crossWasPressed()) {
            robot.drivetrain.setPose(
                    Alliance.current == Alliance.RED ? new Pose(8.1, 7.5, 0) : new Pose(141.5 - 8.1, 7.5, Math.PI)
            );
        }

        if (gamepad2.circleWasPressed()) {
            robot.drivetrain.setPose(robot.drivetrain.getPose().withHeading(
                    Alliance.current == Alliance.RED ? 0 : Math.PI
            ));
        }

        if (gamepad2.dpadLeftWasPressed()) {
            robot.turret.moveLeft();
        }

        if (gamepad2.dpadRightWasPressed()) {
            robot.turret.moveRight();
        }

        if (robot.spindexer.getBallCount() == 3) {
            gamepad2.rumble(100);
        }
    }

    private void setAllianceLed() {
        if (Alliance.current == Alliance.RED) {
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }
}

package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.math.TractorBeam;
import org.firstinspires.ftc.teamcode.math.TurretLocation;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.util.Constants;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

@TeleOp(name = "TeleOp", group = "Competition")
public class TeleOp_Solo extends RobotOpMode {
    private static final double FIELD_WIDTH = 141.5;
    private static final double TRIGGER_THRESHOLD = 0.25;
    private static final double MANUAL_INDEX_TRIGGER_THRESHOLD = 0.85;
    private static final long MANUAL_INDEX_LOCKOUT_MS = 500;

    private boolean teleOpEnabled = false;
    private boolean intakeEnabled = false;
    private boolean turretOffsetControlUnlocked = false;

    private boolean gamepad1StartWasDown = false;
    private boolean gamepad1TouchpadWasDown = false;
    private boolean gamepad1LeftTriggerWasDown = false;
    private boolean gamepad1RightTriggerWasDown = false;
    private boolean gamepad2LeftTriggerWasDown = false;
    private boolean gamepad2RightTriggerWasDown = false;
    private boolean gamepad2TouchpadWasDown = false;
    private long lastLoopNanos = System.nanoTime();

    @Override
    public void init() {
        super.init();

        robot.drivetrain.usePreviousStartingPose();
        robot.drivetrain.startTeleOpDrive();
        robot.drivetrain.setFieldCentricEnabled(true);
        robot.drivetrain.clearFieldCentricHeadingReset();
        robot.turret.usePreviousStartingAngle();
        robot.turret.enableAutoAim();
        robot.intake.off().schedule();
        robot.spindexer.setIntaking(false).schedule();
        intakeEnabled = false;
        setAllianceLed();
    }

    @Override
    public void start() {
        teleOpEnabled = false;
        robot.spindexer.resetEncoderZero();
        lastLoopNanos = System.nanoTime();
    }

    @Override
    public void loop() {
        boolean start = gamepad1.start;
        if (start && !gamepad1StartWasDown) {
            teleOpEnabled = true;
            gamepad1.rumble(2000);
        }
        gamepad1StartWasDown = start;

        handleFieldCentricReset();
        handleAllianceToggle();

        if (robot.turret.autoAimEnabled) {
            TractorBeam.aimTurret(robot.drivetrain.getPose(), robot, Alliance.current);
        }

        if (teleOpEnabled) {
            runDriverControls();
            runOperatorControls();
        }

        Pose turretPose = TurretLocation.getTurretPose(robot.drivetrain.getPose());
        robot.telemetry.addData("TeleOp Enabled", teleOpEnabled);
        robot.telemetry.addData("Turret X", turretPose.getX());
        robot.telemetry.addData("Turret Y", turretPose.getY());
        robot.telemetry.addData("Turret Offset Unlocked", turretOffsetControlUnlocked);
        robot.telemetry.addData("Turret Aim Offset", Constants.turretAimOffsetDegrees);
        robot.telemetry.addData("Alliance", Alliance.current);
        robot.telemetry.addData("Target X", Alliance.current.getGoal().getX());
        robot.telemetry.addData("Target Y", Alliance.current.getGoal().getY());
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

        boolean leftTrigger = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        if (leftTrigger && !gamepad1LeftTriggerWasDown) {
            toggleIntake();
        }
        gamepad1LeftTriggerWasDown = leftTrigger;

        boolean rightTrigger = gamepad1.right_trigger > TRIGGER_THRESHOLD;
        if (rightTrigger && !gamepad1RightTriggerWasDown
                && robot.shooter.isOn()
                && robot.shooter.atTarget()) {
            scheduleShoot();
        }
        gamepad1RightTriggerWasDown = rightTrigger;

        if (gamepad1.rightBumperWasPressed()) {
            scheduleShoot();
        }
    }

    private void runOperatorControls() {
        boolean leftTrigger = gamepad2.left_trigger > TRIGGER_THRESHOLD;
        if (leftTrigger && !gamepad2LeftTriggerWasDown) {
            robot.intake.reverse()
                    .then(robot.spindexer.setIntaking(false))
                    .schedule();
        } else if (!leftTrigger && gamepad2LeftTriggerWasDown) {
            restoreIntakeState();
        }
        gamepad2LeftTriggerWasDown = leftTrigger;

        if (gamepad2.leftBumperWasPressed()) {
            toggleIntake();
        }

        boolean rightTrigger = gamepad2.right_trigger > MANUAL_INDEX_TRIGGER_THRESHOLD;
        if (rightTrigger && !gamepad2RightTriggerWasDown) {
            if (robot.spindexer.canAcceptManualRotation(MANUAL_INDEX_LOCKOUT_MS)) {
                robot.spindexer.rotate120CCWAndIncrementCount().schedule();
            } else {
                gamepad2.rumble(300);
            }
        }
        gamepad2RightTriggerWasDown = rightTrigger;

        if (gamepad2.rightBumperWasPressed()) {
            robot.spindexer.rotate120CCWAndResetCount().schedule();
        }

        if (gamepad2.circleWasPressed()) {
            instant(() -> {
                robot.shooter.useFarInterpolation();
                robot.shooter.turnOn();
                gamepad2.rumble(250);
            }).schedule();
        }

        if (gamepad2.triangleWasPressed()) {
            instant(() -> {
                robot.shooter.useCloseInterpolation();
                robot.shooter.turnOn();
                gamepad2.rumble(250);
            }).schedule();
        }

        if (gamepad2.crossWasPressed()) {
            setGatePose();
            gamepad2.rumble(500);
        }

        if (gamepad2.squareWasPressed()) {
            setHumanPlayerPose();
            gamepad2.rumble(500);
        }

        if (gamepad2.dpadUpWasPressed()) {
            turretOffsetControlUnlocked = !turretOffsetControlUnlocked;
        }

        updateTurretOffsetFromJoystick();

        if (gamepad2.rightStickButtonWasPressed()) {
            Constants.turretAimOffsetDegrees = 0;
        }

        if (gamepad2.dpadLeftWasPressed()) {
            robot.spindexer.nudgeDegrees(-5).schedule();
        }

        if (gamepad2.dpadRightWasPressed()) {
            robot.spindexer.nudgeDegrees(5).schedule();
        }

        if (gamepad2.dpadDownWasPressed()) {
            robot.spindexer.resetManualOffset().schedule();
        }

        if (gamepad2.leftStickButtonWasPressed()) {
            if (robot.shooter.isOn()) {
                robot.shooter.turnOff();
            } else {
                robot.shooter.useCloseInterpolation();
                robot.shooter.turnOn();
            }
        }

        if (robot.spindexer.getBallCount() == 3) {
            gamepad2.rumble(100);
        }
    }

    private void scheduleShoot() {
        robot.spindexer.rotateShootCW()
                .then(waitUntil(() -> !robot.spindexer.isMoving()))
                .then(robot.intake.on(), robot.spindexer.setIntaking(true))
                .schedule();
        intakeEnabled = true;
    }

    private void toggleIntake() {
        intakeEnabled = !intakeEnabled;
        restoreIntakeState();
    }

    private void restoreIntakeState() {
        if (intakeEnabled) {
            robot.intake.on().then(robot.spindexer.setIntaking(true)).schedule();
        } else {
            robot.intake.off().then(robot.spindexer.setIntaking(false)).schedule();
        }
    }

    private void updateTurretOffsetFromJoystick() {
        long now = System.nanoTime();
        double dt = Math.min((now - lastLoopNanos) / 1e9, 0.05);
        lastLoopNanos = now;

        if (!turretOffsetControlUnlocked
                || Math.abs(gamepad2.right_stick_x) < Constants.turretJoystickDeadband) {
            return;
        }

        double nextOffset = Constants.turretAimOffsetDegrees
                + gamepad2.right_stick_x
                * Constants.turretJoystickOffsetRateDegreesPerSecond
                * dt;
        Constants.turretAimOffsetDegrees = Math.max(
                -Constants.turretMaximumAimOffsetDegrees,
                Math.min(Constants.turretMaximumAimOffsetDegrees, nextOffset)
        );
    }

    private void handleFieldCentricReset() {
        if (gamepad1.touchpad && !gamepad1TouchpadWasDown) {
            robot.drivetrain.resetFieldCentricHeading(Alliance.current);
            gamepad1.rumble(500);
        }
        gamepad1TouchpadWasDown = gamepad1.touchpad;
    }

    private void handleAllianceToggle() {
        if (gamepad2.touchpad && !gamepad2TouchpadWasDown) {
            Alliance.current = Alliance.current == Alliance.BLUE ? Alliance.RED : Alliance.BLUE;
            setAllianceLed();
        }
        gamepad2TouchpadWasDown = gamepad2.touchpad;
    }

    private void setGatePose() {
        robot.drivetrain.setPose(
                Alliance.current == Alliance.BLUE
                        ? new Pose(17, 80, Math.PI)
                        : new Pose(FIELD_WIDTH - 17, 80, 0)
        );
    }

    private void setHumanPlayerPose() {
        robot.drivetrain.setPose(
                Alliance.current == Alliance.BLUE
                        ? new Pose(130.5, 6, 0)
                        : new Pose(FIELD_WIDTH - 130.5, 6, Math.PI)
        );
    }

    private void setAllianceLed() {
        if (Alliance.current == Alliance.RED) {
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }
}

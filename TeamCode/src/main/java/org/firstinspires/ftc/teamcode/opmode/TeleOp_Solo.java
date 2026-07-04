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
import static com.pedropathing.ivy.commands.Commands.waitMs;
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
    private boolean spindexerWasFull = false;
    private boolean shooterFarZone = false;
    private boolean readyToShootWasReady = false;

    private boolean gamepad1StartWasDown = false;
    private boolean gamepad1TouchpadWasDown = false;
    private boolean gamepad1LeftTriggerWasDown = false;
    private boolean gamepad1RightTriggerWasDown = false;
    private boolean gamepad2LeftTriggerWasDown = false;
    private boolean gamepad2RightTriggerWasDown = false;
    private boolean gamepad2TouchpadWasDown = false;
    private boolean gatePoseComboWasDown = false;
    private boolean humanPlayerComboWasDown = false;
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

        // Fixed-shot test: spin up at 500 tps with the hood fully out (0.81).
        if (gamepad1.crossWasPressed()) {
            robot.shooter.setTarget(500);
            robot.shooter.setHoodPosition(0.81);
            robot.shooter.turnOn();
        }
    }

    private void runOperatorControls() {
        boolean leftTrigger = gamepad2.left_trigger > TRIGGER_THRESHOLD;
        if (leftTrigger && !gamepad2LeftTriggerWasDown) {
            robot.intake.reverse()
                    .then(robot.spindexer.setIntaking(false))
                    .schedule();
        } else if (!leftTrigger && gamepad2LeftTriggerWasDown) {
            // Releasing a hold-to-reverse always leaves the intake spinning inwards
            // (even if it was stopped before). Left bumper still toggles it off.
            intakeEnabled = true;
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
            if (robot.spindexer.canAcceptManualRotation(MANUAL_INDEX_LOCKOUT_MS)) {
                robot.spindexer.rotate120CCWAndResetCount().schedule();
            } else {
                gamepad2.rumble(300);
            }
        }

        if (gamepad2.circleWasPressed()) {
            shooterFarZone = true;
            instant(() -> {
                robot.shooter.useFarInterpolation();
                robot.shooter.turnOn();
                gamepad2.rumble(250);
            }).schedule();
        }

        if (gamepad2.triangleWasPressed()) {
            shooterFarZone = false;
            instant(() -> {
                robot.shooter.useCloseInterpolation();
                robot.shooter.turnOn();
                gamepad2.rumble(250);
            }).schedule();
        }

        // Relocalize to the gate (y=80) only when Square AND X are held together,
        // so it can't be triggered by an accidental single press.
        boolean gatePoseCombo = gamepad2.square && gamepad2.cross;
        if (gatePoseCombo && !gatePoseComboWasDown) {
            setGatePose();
            gamepad2.rumble(500);
        }
        gatePoseComboWasDown = gatePoseCombo;

        // Relocalize to the human-player position (y=6) only when DPad Left AND
        // DPad Down are held together (a rare combination).
        boolean humanPlayerCombo = gamepad2.dpad_left && gamepad2.dpad_down;
        if (humanPlayerCombo && !humanPlayerComboWasDown) {
            setHumanPlayerPose();
            gamepad2.rumble(500);
        }
        humanPlayerComboWasDown = humanPlayerCombo;

        // Hold DPad Up to unlock left-joystick turret offset control.
        turretOffsetControlUnlocked = gamepad2.dpad_up;

        updateTurretOffsetFromJoystick();

        if (gamepad2.rightStickButtonWasPressed()) {
            Constants.turretAimOffsetDegrees = 0;
        }

        if (gamepad2.leftStickButtonWasPressed()) {
            if (robot.shooter.isOn()) {
                // Off: shooter stops and the hood drops to the bottom.
                robot.shooter.turnOff();
            } else {
                // On: resume whichever interpolation zone was last selected.
                if (shooterFarZone) {
                    robot.shooter.useFarInterpolation();
                } else {
                    robot.shooter.useCloseInterpolation();
                }
                robot.shooter.turnOn();
            }
        }

        handleSpindexerFull();
        handleReadyToShootRumble();
    }

    private void handleReadyToShootRumble() {
        boolean readyToShoot = robot.spindexer.getBallCount() >= 3 && robot.shooter.atTarget();
        if (readyToShoot && !readyToShootWasReady) {
            gamepad2.rumble(400);
        }
        readyToShootWasReady = readyToShoot;
    }

    private void handleSpindexerFull() {
        boolean full = robot.spindexer.getBallCount() >= 3;

        if (full && !spindexerWasFull) {
            // Just filled to 3: warn, hold the intake off, then wait -> reverse to
            // spit the extra ball -> stop. It stays off (intakeEnabled = false)
            // until the driver toggles it or the count drops back below 3.
            gamepad2.rumble(400);
            intakeEnabled = false;
            waitMs(Constants.intakeFullEjectDelayMs)
                    .then(robot.intake.reverse(), robot.spindexer.setIntaking(false))
                    .then(waitMs(Constants.intakeFullEjectReverseMs))
                    .then(robot.intake.off())
                    .schedule();
        } else if (!full && spindexerWasFull && !robot.spindexer.isShooting()) {
            // Dropped below 3 outside of a shot (e.g. manual decrement): resume
            // intaking. During a shot the shoot sequence turns the intake back on
            // itself, so we leave it alone to avoid interrupting the rotation.
            intakeEnabled = true;
            restoreIntakeState();
        }

        spindexerWasFull = full;
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

        double raw = gamepad2.right_stick_x;
        if (!turretOffsetControlUnlocked || Math.abs(raw) < Constants.turretJoystickDeadband) {
            return;
        }

        // Right stick X drives the turret aim offset (same offset the old DPad
        // used). Sign is negated versus that DPad mapping, which turned the turret
        // the wrong way: push right -> turret right, push left -> turret left.
        // An expo curve makes it smooth: barely moving the stick nudges the offset
        // slowly for fine aiming, pulling it hard turns quickly.
        double shaped = Math.signum(raw)
                * Math.pow(Math.abs(raw), Constants.turretJoystickExponent);
        double nextOffset = Constants.turretAimOffsetDegrees
                - shaped
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

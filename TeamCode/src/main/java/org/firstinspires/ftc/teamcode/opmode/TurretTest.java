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

@TeleOp(name = "Turret Test", group = "Tuning")
public class TurretTest extends RobotOpMode {
    private boolean intakeEnabled = false;
    private boolean touchpadWasDown = false;
    private boolean leftTriggerWasDown = false;
    private boolean rightTriggerWasDown = false;
    private boolean dpadLeftWasDown = false;
    private boolean dpadRightWasDown = false;
    private boolean dpadUpWasDown = false;
    private boolean dpadDownWasDown = false;
    private boolean crossWasDown = false;
    private boolean circleWasDown = false;
    private boolean squareWasDown = false;
    private boolean triangleWasDown = false;
    private boolean leftBumperWasDown = false;
    private boolean rightBumperWasDown = false;

    @Override
    public void init() {
        super.init();

        Alliance.current = Alliance.BLUE;
        robot.drivetrain.usePreviousStartingPose();
        robot.drivetrain.startTeleOpDrive();
        robot.drivetrain.setFieldCentricEnabled(true);
        robot.drivetrain.clearFieldCentricHeadingReset();
        robot.turret.usePreviousStartingAngle();
        robot.turret.enableAutoAim();
        robot.intake.off().schedule();
        robot.spindexer.setIntaking(false).schedule();
        setAllianceLed();
    }

    @Override
    public void loop() {
        handleDrive();
        handleAllianceSelection();
        handleTurretControls();
        handleIntakeAndShooterControls();

        if (robot.turret.autoAimEnabled) {
            TractorBeam.aimTurret(robot.drivetrain.getPose(), robot, Alliance.current);
        }

        Pose turretPose = TurretLocation.getTurretPose(robot.drivetrain.getPose());
        robot.telemetry.addData("Alliance", Alliance.current);
        robot.telemetry.addData("Target X", Alliance.current.getGoal().getX());
        robot.telemetry.addData("Target Y", Alliance.current.getGoal().getY());
        robot.telemetry.addData("Turret X", turretPose.getX());
        robot.telemetry.addData("Turret Y", turretPose.getY());
        robot.telemetry.addData("Controls", "L trigger intake, X far shooter, O interpolation shooter, Triangle toggle shooter");
        robot.telemetry.addData("Turret Controls", "Dpad L/R manual, Dpad Up auto aim, Dpad Down turret off");

        super.loop();
    }

    private void handleDrive() {
        robot.drivetrain.fieldCentricDrive(
                -gamepad1.left_stick_y * Constants.driveForwardMultiplier,
                -gamepad1.left_stick_x * Constants.driveStrafeMultiplier,
                -gamepad1.right_stick_x * Constants.driveTurnMultiplier,
                Alliance.current
        );

        if (gamepad1.touchpad && !touchpadWasDown) {
            robot.drivetrain.resetFieldCentricHeading(Alliance.current);
            gamepad1.rumble(250);
        }
        touchpadWasDown = gamepad1.touchpad;
    }

    private void handleAllianceSelection() {
        if (gamepad2.left_bumper && !leftBumperWasDown) {
            Alliance.current = Alliance.RED;
            setAllianceLed();
        }
        leftBumperWasDown = gamepad2.left_bumper;

        if (gamepad2.right_bumper && !rightBumperWasDown) {
            Alliance.current = Alliance.BLUE;
            setAllianceLed();
        }
        rightBumperWasDown = gamepad2.right_bumper;
    }

    private void handleTurretControls() {
        if (gamepad1.dpad_left && !dpadLeftWasDown) {
            robot.turret.moveLeft();
        }
        dpadLeftWasDown = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !dpadRightWasDown) {
            robot.turret.moveRight();
        }
        dpadRightWasDown = gamepad1.dpad_right;

        if (gamepad1.dpad_up && !dpadUpWasDown) {
            robot.turret.enableAutoAim();
        }
        dpadUpWasDown = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !dpadDownWasDown) {
            robot.turret.disableAutoAim();
            robot.turret.off();
        }
        dpadDownWasDown = gamepad1.dpad_down;
    }

    private void handleIntakeAndShooterControls() {
        boolean leftTrigger = gamepad1.left_trigger > 0.25;
        if (leftTrigger && !leftTriggerWasDown) {
            intakeEnabled = !intakeEnabled;
            if (intakeEnabled) {
                robot.intake.on().then(robot.spindexer.setIntaking(true)).schedule();
            } else {
                robot.intake.off().then(robot.spindexer.setIntaking(false)).schedule();
            }
        }
        leftTriggerWasDown = leftTrigger;

        boolean rightTrigger = gamepad1.right_trigger > 0.25;
        if (rightTrigger && !rightTriggerWasDown) {
            robot.spindexer.rotateShootCW()
                    .then(waitUntil(() -> !robot.spindexer.isMoving()))
                    .then(robot.intake.on(), robot.spindexer.setIntaking(true))
                    .schedule();
            intakeEnabled = true;
        }
        rightTriggerWasDown = rightTrigger;

        if (gamepad1.cross && !crossWasDown) {
            instant(() -> {
                robot.shooter.setTarget(Constants.shootVelFar);
                robot.shooter.setHoodPosition(Constants.shootHoodFar);
                robot.shooter.turnOn();
            }).schedule();
        }
        crossWasDown = gamepad1.cross;

        if (gamepad1.circle && !circleWasDown) {
            instant(() -> {
                robot.shooter.useInterpolation();
                robot.shooter.turnOn();
            }).schedule();
        }
        circleWasDown = gamepad1.circle;

        if (gamepad1.triangle && !triangleWasDown) {
            robot.shooter.toggle();
        }
        triangleWasDown = gamepad1.triangle;

        if (gamepad1.square && !squareWasDown) {
            instant(robot.shooter::turnOff).schedule();
        }
        squareWasDown = gamepad1.square;
    }

    private void setAllianceLed() {
        if (Alliance.current == Alliance.RED) {
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }
}

package org.firstinspires.ftc.teamcode.opmode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.math.TractorBeam;
import org.firstinspires.ftc.teamcode.math.TurretLocation;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;
import org.firstinspires.ftc.teamcode.util.Constants;

@Configurable
@TeleOp(name = "Shooter Interpolation Tuning", group = "Tuning")
public class ShooterInterpolationTuning extends RobotOpMode {
    private static final double[] FAR_X = new double[]{43, 71, 100};
    private static final double[] FAR_Y = new double[]{6, 27};
    private static final double[] CLOSE_X = new double[]{38, 61, 85};
    private static final double[] CLOSE_Y = new double[]{63, 88, 111, 135.5};
    private static final double FIELD_WIDTH = 141.5;

    public static double tuningVelocity = 1500;
    public static double tuningHood = 0.38;
    public static double velocityStep = 25;
    public static double hoodStep = 0.01;
    public static int selectedPoint = 0;
    public static boolean shooterEnabled = true;
    public static boolean tuneBlueSide = true;
    public static boolean saveCurrentPoint = false;
    public static boolean nextPoint = false;
    public static boolean previousPoint = false;

    private static final int POINT_COUNT = FAR_X.length * FAR_Y.length + CLOSE_X.length * CLOSE_Y.length;
    private final double[] savedVelocity = new double[POINT_COUNT];
    private final double[] savedHood = new double[POINT_COUNT];
    private final boolean[] saved = new boolean[POINT_COUNT];

    private boolean touchpadWasDown = false;
    private boolean dpadLeftWasDown = false;
    private boolean dpadRightWasDown = false;
    private boolean dpadUpWasDown = false;
    private boolean dpadDownWasDown = false;
    private boolean triangleWasDown = false;
    private boolean crossWasDown = false;
    private boolean squareWasDown = false;
    private boolean circleWasDown = false;
    private boolean leftBumperWasDown = false;
    private boolean rightBumperWasDown = false;

    @Override
    public void init() {
        super.init();
        PanelsConfigurables.INSTANCE.refreshClass(this);

        Alliance.current = tuneBlueSide ? Alliance.BLUE : Alliance.RED;
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
        Alliance.current = tuneBlueSide ? Alliance.BLUE : Alliance.RED;
        handleDrive();
        handleAllianceSelection();
        handleTuningControls();

        Pose turretPose = TurretLocation.getTurretPose(robot.drivetrain.getPose());
        if (robot.turret.autoAimEnabled) {
            TractorBeam.aimTurret(robot.drivetrain.getPose(), robot, Alliance.current);
        }

        robot.shooter.setTarget(tuningVelocity);
        robot.shooter.setHoodPosition(tuningHood);
        if (shooterEnabled) {
            robot.shooter.turnOn();
        } else {
            robot.shooter.turnOff();
        }

        addTelemetry(turretPose);
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
            tuneBlueSide = false;
            Alliance.current = Alliance.RED;
            setAllianceLed();
        }
        leftBumperWasDown = gamepad2.left_bumper;

        if (gamepad2.right_bumper && !rightBumperWasDown) {
            tuneBlueSide = true;
            Alliance.current = Alliance.BLUE;
            setAllianceLed();
        }
        rightBumperWasDown = gamepad2.right_bumper;
    }

    private void handleTuningControls() {
        selectedPoint = clampIndex(selectedPoint);

        if (previousPoint) {
            selectedPoint = clampIndex(selectedPoint - 1);
            previousPoint = false;
        }

        if (nextPoint) {
            selectedPoint = clampIndex(selectedPoint + 1);
            nextPoint = false;
        }

        if (gamepad2.dpad_left && !dpadLeftWasDown) {
            selectedPoint = clampIndex(selectedPoint - 1);
        }
        dpadLeftWasDown = gamepad2.dpad_left;

        if (gamepad2.dpad_right && !dpadRightWasDown) {
            selectedPoint = clampIndex(selectedPoint + 1);
        }
        dpadRightWasDown = gamepad2.dpad_right;

        if (gamepad2.dpad_up && !dpadUpWasDown) {
            tuningVelocity += velocityStep;
        }
        dpadUpWasDown = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !dpadDownWasDown) {
            tuningVelocity = Math.max(0, tuningVelocity - velocityStep);
        }
        dpadDownWasDown = gamepad2.dpad_down;

        if (gamepad2.triangle && !triangleWasDown) {
            tuningHood = clipHood(tuningHood + hoodStep);
        }
        triangleWasDown = gamepad2.triangle;

        if (gamepad2.cross && !crossWasDown) {
            tuningHood = clipHood(tuningHood - hoodStep);
        }
        crossWasDown = gamepad2.cross;

        if (saveCurrentPoint || (gamepad2.square && !squareWasDown)) {
            savePoint();
            saveCurrentPoint = false;
            gamepad2.rumble(250);
        }
        squareWasDown = gamepad2.square;

        if (gamepad2.circle && !circleWasDown) {
            shooterEnabled = !shooterEnabled;
        }
        circleWasDown = gamepad2.circle;
    }

    private void savePoint() {
        int tableIndex = getTableIndex(selectedPoint);
        savedVelocity[tableIndex] = tuningVelocity;
        savedHood[tableIndex] = tuningHood;
        saved[tableIndex] = true;
    }

    private void addTelemetry(Pose turretPose) {
        int tableIndex = getTableIndex(selectedPoint);
        double pointX = getDrivePointX(selectedPoint);
        double pointY = getPointY(selectedPoint);
        double tableX = getTablePointX(tableIndex);
        double tableY = getTablePointY(tableIndex);
        double dx = pointX - robot.drivetrain.getPose().getX();
        double dy = pointY - robot.drivetrain.getPose().getY();
        boolean pointSaved = saved[tableIndex];

        robot.telemetry.addData("Alliance", Alliance.current);
        robot.telemetry.addData("Selected Point", "%d/%d %s", selectedPoint + 1, POINT_COUNT, getPointName(selectedPoint));
        robot.telemetry.addData("Drive To X", pointX);
        robot.telemetry.addData("Drive To Y", pointY);
        robot.telemetry.addData("WaveLength Table X", tableX);
        robot.telemetry.addData("WaveLength Table Y", tableY);
        robot.telemetry.addData("Pose X", robot.drivetrain.getPose().getX());
        robot.telemetry.addData("Pose Y", robot.drivetrain.getPose().getY());
        robot.telemetry.addData("Point Error", "dx %.1f, dy %.1f", dx, dy);
        robot.telemetry.addData("Turret X", turretPose.getX());
        robot.telemetry.addData("Turret Y", turretPose.getY());
        robot.telemetry.addData("Tuning Velocity", tuningVelocity);
        robot.telemetry.addData("Tuning Hood", tuningHood);
        robot.telemetry.addData("Shooter Enabled", shooterEnabled);
        robot.telemetry.addData("Point Saved", pointSaved);
        robot.telemetry.addData("Current Java Entry", "table x %.1f, y %.1f -> velocity %.0f, hood %.3f", tableX, tableY, tuningVelocity, tuningHood);
        robot.telemetry.addData("Controls", "g2 dpad L/R point, dpad U/D velocity, triangle/cross hood, square save, circle shooter");
        robot.telemetry.addData("Panels", "selectedPoint, tuningVelocity, tuningHood, tuneBlueSide, saveCurrentPoint, nextPoint, previousPoint");
        robot.telemetry.addData("Saved Count", getSavedCount());

        if (pointSaved) {
            robot.telemetry.addData("Saved Java Entry", "table x %.1f, y %.1f -> velocity %.0f, hood %.3f",
                    tableX, tableY, savedVelocity[tableIndex], savedHood[tableIndex]);
        }
    }

    private int getSavedCount() {
        int count = 0;
        for (boolean pointSaved : saved) {
            if (pointSaved) count++;
        }
        return count;
    }

    private int clampIndex(int index) {
        if (index < 0) return POINT_COUNT - 1;
        if (index >= POINT_COUNT) return 0;
        return index;
    }

    private double clipHood(double hood) {
        return Math.max(Constants.adjHoodMax, Math.min(Constants.adjHoodMin, hood));
    }

    private String getPointName(int index) {
        return index < FAR_X.length * FAR_Y.length ? "FAR" : "CLOSE";
    }

    private int getTableIndex(int displayIndex) {
        if (Alliance.current != Alliance.BLUE) {
            return displayIndex;
        }

        if (displayIndex < FAR_X.length * FAR_Y.length) {
            int row = displayIndex / FAR_X.length;
            int col = displayIndex % FAR_X.length;
            return row * FAR_X.length + (FAR_X.length - 1 - col);
        }

        int closeDisplayIndex = displayIndex - FAR_X.length * FAR_Y.length;
        int row = closeDisplayIndex / CLOSE_X.length;
        int col = closeDisplayIndex % CLOSE_X.length;
        return FAR_X.length * FAR_Y.length + row * CLOSE_X.length + (CLOSE_X.length - 1 - col);
    }

    private double getDrivePointX(int displayIndex) {
        double tableX = getTablePointX(getTableIndex(displayIndex));
        return Alliance.current == Alliance.BLUE ? FIELD_WIDTH - tableX : tableX;
    }

    private double getTablePointX(int tableIndex) {
        if (tableIndex < FAR_X.length * FAR_Y.length) {
            return FAR_X[tableIndex % FAR_X.length];
        }

        int closeIndex = tableIndex - FAR_X.length * FAR_Y.length;
        return CLOSE_X[closeIndex % CLOSE_X.length];
    }

    private double getPointY(int displayIndex) {
        return getTablePointY(getTableIndex(displayIndex));
    }

    private double getTablePointY(int tableIndex) {
        if (tableIndex < FAR_X.length * FAR_Y.length) {
            return FAR_Y[tableIndex / FAR_X.length];
        }

        int closeIndex = tableIndex - FAR_X.length * FAR_Y.length;
        return CLOSE_Y[closeIndex / CLOSE_X.length];
    }

    private void setAllianceLed() {
        if (Alliance.current == Alliance.RED) {
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }
}

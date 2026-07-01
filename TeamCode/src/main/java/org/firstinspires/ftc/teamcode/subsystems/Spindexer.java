package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.HardwareNames;

import java.util.concurrent.TimeUnit;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

public class Spindexer {
    private final Robot robot;
    private final DcMotorEx spindexerMotor;
    private final DcMotorEx spindexerEncoder;
    private final DigitalChannel ranger;
    private final Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();

    private double targetPositionTicks = 0;

    private boolean ignoreUnstuck = false;
    private long lastUnstuckTime = 0;

    private boolean canRotate = false;
    private boolean lastBallDetected = false;
    private boolean ballDetected = false;
    private boolean ignoreSensor = false;
    private long sensorWait = Constants.sensorWait;
    private long lastDetectTime = 0;
    private boolean ballDetectionLatched = false;
    private boolean autoLoadPending = false;
    private long autoLoadTime = 0;
    private int ballCount = 0;

    private boolean intaking = false;
    private boolean shooting = false;
    private boolean intakeStuck = false;
    private boolean jamIndexPending = false;
    private long intakeHighCurrentStartTime = -1;
    private boolean activeMove = false;
    private boolean targetSettled = false;
    private boolean wasMoving = false;
    private long moveStartTime = 0;
    private int encoderZeroTicks = 0;
    private double positionIntegral = 0;
    private double lastPositionError = 0;
    private long lastPidTimeNanos = System.nanoTime();
    private double lastPidOutput = 0;

    public Spindexer(Robot robot) {
        this.robot = robot;
        spindexerMotor = robot.hardwareMap.get(DcMotorEx.class, HardwareNames.spindexer);
        spindexerEncoder = robot.hardwareMap.get(DcMotorEx.class, HardwareNames.spindexerEncoder);
        ranger = robot.hardwareMap.get(DigitalChannel.class, HardwareNames.ranger);
        telemetry = robot.telemetry;

        ranger.setMode(DigitalChannel.Mode.INPUT);

        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.setPower(0);

        // Do not STOP_AND_RESET this encoder if it is plugged into a drive motor port.
        encoderZeroTicks = spindexerEncoder.getCurrentPosition();
        targetPositionTicks = getCurrentPosition();
    }

    public Command rotate360CW() {
        return instant(() -> {
            moveRelative(-Constants.spindexerTicksPerRevolution);
            ballCount = 0;
        }).requiring(spindexerMotor);
    }

    public Command rotateShootCW() {
        return instant(() -> {
            intaking = false;
            shooting = true;
            moveRelative(-(Constants.spindexerTicksPerRevolution + ticks120Degrees()));
            ballCount = 0;
        }).requiring(spindexerMotor);
    }

    public Command rotateResetCW() {
        return instant(() -> {
            moveRelative(ticks120Degrees() - 20);
            ballCount = 0;
        }).requiring(spindexerMotor);
    }

    public Command rotate360CCW() {
        return instant(() -> moveRelative(Constants.spindexerTicksPerRevolution)).requiring(spindexerMotor);
    }

    public Command rotate15CW() {
        return instant(() -> {
            moveRelative(Constants.spindexerTicksPerRevolution / 15.0);
            ballCount--;
        }).requiring(spindexerMotor);
    }

    public Command rotate120CCW() {
        return instant(() -> moveRelative(ticks120Degrees())).requiring(spindexerMotor);
    }

    public Command rotate120CW() {
        return instant(() -> {
            moveRelative(-ticks120Degrees());
            ignoreSensor(Constants.shootSingleSensorWait);
            ballCount--;
        }).requiring(spindexerMotor);
    }

    public Command stop() {
        return instant(() -> {
            activeMove = false;
            targetPositionTicks = getCurrentPosition();
            spindexerMotor.setPower(0);
            resetPositionController();
        }).requiring(spindexerMotor);
    }

    public Command resetBallCount() {
        return instant(() -> ballCount = 0);
    }

    public Command decrementBallCount() {
        return instant(() -> ballCount--);
    }

    public Command setIntaking(boolean intaking) {
        return instant(() -> {
            if (intaking && shooting) {
                ballDetectionLatched = false;
                autoLoadPending = false;
                ignoreSensor(Constants.postShootSensorWaitMs);
            }
            this.intaking = intaking;
            this.shooting = false;
            intakeHighCurrentStartTime = -1;
            if (!intaking) {
                ballDetectionLatched = false;
                autoLoadPending = false;
                jamIndexPending = false;
            }
        });
    }

    public boolean isMoving() {
        return activeMove
                && !targetSettled
                && Math.abs(targetPositionTicks - getCurrentPosition()) > deadbandTicks();
    }

    public int getCurrentPosition() {
        int direction = Constants.spindexerEncoderReversed ? -1 : 1;
        return direction * (spindexerEncoder.getCurrentPosition() - encoderZeroTicks);
    }

    public int getTargetPosition() {
        return (int) Math.round(targetPositionTicks);
    }

    public int getBallCount() {
        return ballCount;
    }

    public void resetEncoderZero() {
        spindexerMotor.setPower(0);
        encoderZeroTicks = spindexerEncoder.getCurrentPosition();
        targetPositionTicks = 0;
        activeMove = false;
        targetSettled = false;
        autoLoadPending = false;
        jamIndexPending = false;
        resetPositionController();
    }

    public double getCurrent() {
        return spindexerMotor.getCurrent(CurrentUnit.MILLIAMPS);
    }

    private void moveRelative(double deltaTicks) {
        targetPositionTicks += deltaTicks;
        activeMove = true;
        targetSettled = false;
        resetPositionController();
    }

    private void ignoreSensor(long waitMs) {
        ignoreSensor = true;
        sensorWait = waitMs;
        lastDetectTime = timer.time(TimeUnit.MILLISECONDS);
    }

    private double ticks120Degrees() {
        return Constants.spindexerTicksPerRevolution / 3.0;
    }

    private double deadbandTicks() {
        return (Constants.spindexerDeadbandDegrees / 360.0) * Constants.spindexerTicksPerRevolution;
    }

    private double holdCorrectionTicks() {
        return (Constants.spindexerHoldCorrectionDegrees / 360.0)
                * Constants.spindexerTicksPerRevolution;
    }

    private double nearestSlotPosition(double positionTicks) {
        double slotTicks = ticks120Degrees();
        return Math.round(positionTicks / slotTicks) * slotTicks;
    }

    private void snapTargetToNearestSlot() {
        targetPositionTicks = nearestSlotPosition(getCurrentPosition());
        resetPositionController();
    }

    private void restartPositionControllerAfterTimeout() {
        activeMove = true;
        targetSettled = false;
        moveStartTime = timer.time(TimeUnit.MILLISECONDS);
        resetPositionController();
    }

    private void resetPositionController() {
        positionIntegral = 0;
        lastPositionError = targetPositionTicks - getCurrentPosition();
        lastPidTimeNanos = System.nanoTime();
        lastPidOutput = 0;
    }

    private void updateMotorPower() {
        if (!activeMove) {
            spindexerMotor.setPower(0);
            lastPidOutput = 0;
            return;
        }

        long now = System.nanoTime();
        double dt = (now - lastPidTimeNanos) / 1e9;
        if (dt <= 0 || dt > 0.25) dt = 0.02;

        double error = targetPositionTicks - getCurrentPosition();
        if (targetSettled && Math.abs(error) <= holdCorrectionTicks()) {
            spindexerMotor.setPower(0);
            lastPidOutput = 0;
            return;
        }

        if (targetSettled) {
            targetSettled = false;
            resetPositionController();
            error = targetPositionTicks - getCurrentPosition();
        }

        if (Math.abs(error) <= deadbandTicks()) {
            targetSettled = true;
            spindexerMotor.setPower(0);
            resetPositionController();
            return;
        }

        positionIntegral += error * dt;
        positionIntegral = Range.clip(
                positionIntegral,
                -Constants.spindexerMaxIntegral,
                Constants.spindexerMaxIntegral
        );

        double derivative = (error - lastPositionError) / dt;
        lastPositionError = error;
        lastPidTimeNanos = now;

        double power = Range.clip(
                error * Constants.spindexerPositionKp
                        + positionIntegral * Constants.spindexerPositionKi
                        + derivative * Constants.spindexerPositionKd,
                -1.0,
                1.0
        );

        if (Constants.spindexerMotorReversed) {
            power *= -1.0;
        }

        lastPidOutput = power;
        spindexerMotor.setPower(power);
    }

    private void autoLoadBall() {
        if (!autoLoadPending || !intaking || shooting || isMoving()) {
            return;
        }

        autoLoadPending = false;
        ballCount++;
        if (ballCount < 3) {
            moveRelative(ticks120Degrees());
        } else {
            intaking = false;
        }
    }

    public Command periodic() {
        return infinite(() -> {
            long now = timer.time(TimeUnit.MILLISECONDS);

            updateMotorPower();

            boolean moving = isMoving();
            if (moving && !wasMoving) {
                moveStartTime = now;
            }

            if (!moving && wasMoving) {
                lastBallDetected = ballDetected;
                jamIndexPending = false;
            }

            if (moving && now - moveStartTime > Constants.spindexerMoveTimeoutMs) {
                restartPositionControllerAfterTimeout();
            }

            wasMoving = moving;

            canRotate = ranger.getState();
            ballDetected = canRotate;

            if (ballDetected
                    && !lastBallDetected
                    && Constants.autoSpindex
                    && intaking
                    && !ignoreSensor) {
                ballDetectionLatched = true;
            }

            if (ballDetectionLatched
                    && Constants.autoSpindex
                    && intaking
                    && !ignoreSensor
                    && !moving
                    && !autoLoadPending) {
                ballDetectionLatched = false;
                ignoreSensor(Constants.sensorWait);
                autoLoadPending = true;
                autoLoadTime = now + Constants.spindexerAutoLoadDelayMs;
            }

            if (autoLoadPending && now >= autoLoadTime && !moving) {
                autoLoadBall();
            }

            double spindexerCurrent = getCurrent();
            double intakeCurrent = robot.intake.getIntakeCurrent();
            if (intaking
                    && !shooting
                    && robot.intake.isOn()
                    && intakeCurrent >= Constants.intakeStuckCurrentMilliamps) {
                if (intakeHighCurrentStartTime < 0) {
                    intakeHighCurrentStartTime = now;
                }
            } else {
                intakeHighCurrentStartTime = -1;
            }

            boolean intakeCurrentSustained =
                    intakeHighCurrentStartTime >= 0
                            && now - intakeHighCurrentStartTime
                            >= Constants.intakeStuckDetectionTimeMs;

            if (intaking
                    && !shooting
                    && intakeCurrentSustained
                    && !ignoreUnstuck) {
                intakeStuck = true;
                ignoreUnstuck = true;
                lastUnstuckTime = now;
                intakeHighCurrentStartTime = -1;

                if (!autoLoadPending && !jamIndexPending && !isMoving()) {
                    jamIndexPending = true;
                    robot.intake.shortReverse()
                            .then(rotate120CCW())
                            .schedule();
                } else {
                    robot.intake.shortReverse().schedule();
                }
            }

            if (now - lastUnstuckTime >= Constants.spindexerUnstuckWaitMs) {
                ignoreUnstuck = false;
                intakeStuck = false;
            }

            if (now - lastDetectTime >= sensorWait) {
                ignoreSensor = false;
            }

            lastBallDetected = ballDetected;

            telemetry.addData("Spindexer Position", getCurrentPosition());
            telemetry.addData("Spindexer Raw Encoder", spindexerEncoder.getCurrentPosition());
            telemetry.addData("Spindexer Encoder Zero", encoderZeroTicks);
            telemetry.addData("Spindexer Target", getTargetPosition());
            telemetry.addData("Spindexer Moving", isMoving());
            telemetry.addData("Spindexer Current mA", spindexerCurrent);
            telemetry.addData("Intake Current mA", intakeCurrent);
            telemetry.addData("Spindexer Motor Power", spindexerMotor.getPower());
            telemetry.addData("Spindexer PID Output", lastPidOutput);
            telemetry.addData("Spindexer Intake Stuck", intakeStuck);
            telemetry.addData("Spindexer Jam Index Pending", jamIndexPending);
            telemetry.addData("Intake High Current Time",
                    intakeHighCurrentStartTime < 0 ? 0 : now - intakeHighCurrentStartTime);
            telemetry.addData("Spindexer Ranger Can Rotate", canRotate);
            telemetry.addData("Spindexer Ball Count", ballCount);
            telemetry.addData("Spindexer Ignore Sensor", ignoreSensor);
            telemetry.addData("Spindexer Ball Detection Latched", ballDetectionLatched);
            telemetry.addData("Spindexer Auto Load Pending", autoLoadPending);
            telemetry.addData("Spindexer Intaking", intaking);
            telemetry.addData("Spindexer Shooting", shooting);
        });
    }
}

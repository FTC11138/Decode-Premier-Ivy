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
    private boolean ballDetected = false;
    private boolean ignoreSensor = false;
    private long sensorWait = Constants.sensorWait;
    private long lastDetectTime = 0;
    private boolean autoLoadPending = false;
    private long autoLoadTime = 0;
    private boolean autoIndexArmed = true;
    private int ballCount = 0;

    private boolean intaking = false;
    private boolean shooting = false;
    private boolean intakeStuck = false;
    private boolean jamIndexPending = false;
    private long intakeHighCurrentStartTime = -1;
    private long spindexerHighCurrentStartTime = -1;
    private boolean activeMove = false;
    private boolean targetSettled = false;
    private boolean wasMoving = false;
    private long moveStartTime = 0;
    private long lastMoveCommandTimeMs = Long.MIN_VALUE;
    private double manualOffsetTicks = 0;
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
        return instant(() -> moveOne120DegreeSlot(1)).requiring(spindexerMotor);
    }

    public Command rotate120CCWAndIncrementCount() {
        return instant(() -> {
            moveOne120DegreeSlot(1);
            if (ballCount < 3) {
                ballCount++;
            }
        }).requiring(spindexerMotor);
    }

    public Command rotate120CCWAndResetCount() {
        return instant(() -> {
            moveOne120DegreeSlot(1);
            ballCount = 0;
        }).requiring(spindexerMotor);
    }

    public Command nudgeDegrees(double degrees) {
        return instant(() -> {
            double ticks = degrees / 360.0 * Constants.spindexerTicksPerRevolution;
            manualOffsetTicks += ticks;
            moveRelative(ticks);
        }).requiring(spindexerMotor);
    }

    public Command resetManualOffset() {
        return instant(() -> {
            if (Math.abs(manualOffsetTicks) > 1e-6) {
                moveRelative(-manualOffsetTicks);
            }
            manualOffsetTicks = 0;
        }).requiring(spindexerMotor);
    }

    public Command rotate120CW() {
        return instant(() -> {
            moveOne120DegreeSlot(-1);
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
                autoLoadPending = false;
                ignoreSensor(Constants.postShootSensorWaitMs);
            }
            this.intaking = intaking;
            this.shooting = false;
            intakeHighCurrentStartTime = -1;
            if (!intaking) {
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

    /**
     * True while the spindexer is actively driving counterclockwise - the
     * indexing/intake direction (positive target), not the shooting direction
     * (which rotates clockwise / negative).
     */
    public boolean isSpinningCounterClockwise() {
        return isMoving() && targetPositionTicks - getCurrentPosition() > 0;
    }

    /**
     * True while the spindexer is actively driving clockwise - the shooting
     * direction (negative target).
     */
    public boolean isSpinningClockwise() {
        return isMoving() && targetPositionTicks - getCurrentPosition() < 0;
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

    public boolean isShooting() {
        return shooting;
    }

    public boolean canAcceptManualRotation(long lockoutMs) {
        return rotationLockoutElapsed(lockoutMs);
    }

    // True once at least lockoutMs has passed since the last commanded rotation
    // (manual or automatic). Gates both manual and auto spins so neither happens
    // within the lockout window of another. Not gated on isMoving(): the position
    // controller's small hold corrections would otherwise block every rotation.
    private boolean rotationLockoutElapsed(long lockoutMs) {
        if (lastMoveCommandTimeMs == Long.MIN_VALUE) {
            return true;
        }
        return System.currentTimeMillis() - lastMoveCommandTimeMs >= lockoutMs;
    }

    public void resetEncoderZero() {
        spindexerMotor.setPower(0);
        encoderZeroTicks = spindexerEncoder.getCurrentPosition();
        targetPositionTicks = 0;
        manualOffsetTicks = 0;
        activeMove = false;
        targetSettled = false;
        autoLoadPending = false;
        jamIndexPending = false;
        autoIndexArmed = true;
        resetPositionController();
    }

    public double getCurrent() {
        return spindexerMotor.getCurrent(CurrentUnit.MILLIAMPS);
    }

    private void moveRelative(double deltaTicks) {
        targetPositionTicks += deltaTicks;
        lastMoveCommandTimeMs = System.currentTimeMillis();
        activeMove = true;
        targetSettled = false;
        resetPositionController();
    }

    private void moveOne120DegreeSlot(int direction) {
        double slotTicks = ticks120Degrees();
        double positionWithoutOffset = getCurrentPosition() - manualOffsetTicks;
        long nearestSlot = Math.round(positionWithoutOffset / slotTicks);
        targetPositionTicks =
                (nearestSlot + direction) * slotTicks + manualOffsetTicks;
        lastMoveCommandTimeMs = System.currentTimeMillis();
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

        // Static feedforward in the direction of travel to break past the detent.
        // Reached only when |error| > deadband, so it never applies at rest.
        double feedforward = Math.signum(error) * Constants.spindexerFeedforward;
        double power = Range.clip(
                error * Constants.spindexerPositionKp
                        + positionIntegral * Constants.spindexerPositionKi
                        + derivative * Constants.spindexerPositionKd
                        + feedforward,
                -1.0,
                1.0
        );

        if (Constants.spindexerMotorReversed) {
            power *= -1.0;
        }

        lastPidOutput = power;
        spindexerMotor.setPower(power);
    }

    private void queueAutoLoad(long now) {
        // Note: the sensor debounce is applied when the rotation actually finishes
        // (see periodic), not here at schedule time.
        autoLoadPending = true;
        autoLoadTime = now + Constants.spindexerAutoLoadDelayMs;
    }

    private void autoLoadBall() {
        if (!autoLoadPending || !intaking || shooting || isMoving()) {
            return;
        }

        autoLoadPending = false;
        if (ballCount >= 3) {
            // Already full - never over-fill or rotate past the last slot.
            intaking = false;
            return;
        }
        ballCount++;
        if (ballCount < 3) {
            // Rotate the next open slot to the intake.
            moveOne120DegreeSlot(1);
        } else {
            // Third ball just seated: spindexer full, stop auto-indexing.
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
                jamIndexPending = false;
                // Debounce the sensor from the moment the rotation actually
                // finishes, so the just-moved ball isn't immediately re-detected.
                ignoreSensor(Constants.sensorWait);
            }

            if (moving && now - moveStartTime > Constants.spindexerMoveTimeoutMs) {
                restartPositionControllerAfterTimeout();
            }

            wasMoving = moving;

            canRotate = ranger.getState();
            ballDetected = canRotate;

            // Re-arm once the sensor clears: the previously loaded ball has moved
            // off the sensor, so we are ready for the next one. Requiring this
            // clear before another turn is what prevents a second turn on a ball
            // that is still sitting on / transiting the sensor from the previous
            // turn - which would push an empty slot through the intake. Re-arming
            // still happens during the ignore/lockout window, so a new ball that
            // shows up after the old one clears is still caught once the window
            // ends.
            if (!ballDetected) {
                autoIndexArmed = true;
            }

            // Index a fresh ball: a ball is seated, we are armed (the sensor has
            // cleared since the last turn), and it is safe to act. Both timers are
            // anchored to the real rotation - the ignore debounce starts when the
            // turn finishes, the lockout is measured from when the turn starts.
            if (ballDetected
                    && autoIndexArmed
                    && Constants.autoSpindex
                    && intaking
                    && !ignoreSensor
                    && !moving
                    && !autoLoadPending
                    && ballCount < 3
                    && rotationLockoutElapsed(Constants.spindexerRotationLockoutMs)) {
                autoIndexArmed = false;
                queueAutoLoad(now);
            }

            if (autoLoadPending && now >= autoLoadTime && !moving) {
                autoLoadBall();
            }

            double spindexerCurrent = getCurrent();
            double intakeCurrent = robot.intake.getIntakeCurrent();
            boolean intakeOn = robot.intake.isOn();

            // Time high intake current whenever the motor is on (intaking OR
            // shooting). Start at the trigger threshold, keep timing until current
            // drops below the release threshold (hysteresis) so noise near the
            // trigger doesn't keep resetting it.
            double holdThreshold = intakeHighCurrentStartTime < 0
                    ? Constants.intakeStuckCurrentMilliamps
                    : Constants.intakeStuckReleaseMilliamps;
            if (intakeOn && intakeCurrent >= holdThreshold) {
                if (intakeHighCurrentStartTime < 0) {
                    intakeHighCurrentStartTime = now;
                }
            } else {
                intakeHighCurrentStartTime = -1;
            }

            long highCurrentDuration =
                    intakeHighCurrentStartTime < 0 ? 0 : now - intakeHighCurrentStartTime;

            // Normal jam: current sustained while intaking (not shooting) and the
            // auto-index path is NOT busy (spindexer idle, no ball seated/queued).
            // That guard keeps a normal ball load - which also spikes current -
            // from triggering a reverse that fights the intake.
            boolean handlingBall = moving
                    || autoLoadPending
                    || ballDetected;
            boolean normalJam =
                    highCurrentDuration >= Constants.intakeStuckDetectionTimeMs && !handlingBall;
            // Hard jam: current high far longer than any normal load, even while the
            // path looks busy - the ball clearly isn't clearing.
            boolean hardJam = highCurrentDuration >= Constants.intakeHardJamTimeMs;
            // Never run the reverse+index jam response when already full - that is
            // how a 4th ball was being accepted. At 3 balls the spindexer must not
            // turn or count from a jam.
            boolean intakeMotorJam =
                    intaking && !shooting && intakeOn && ballCount < 3 && (normalJam || hardJam);
            // At 3 balls a jam still needs to spit the wedged extra ball back out so
            // it doesn't get stuck, but must NOT turn the spindexer or change count.
            boolean fullJamReverse =
                    intaking && !shooting && intakeOn && ballCount >= 3 && (normalJam || hardJam);

            // Spindexer strained mid-CCW-turn: the spindexer motor is drawing high
            // current while trying to turn counterclockwise (a ball wedging it).
            // Time it (sustained) so the normal acceleration spike doesn't trip it.
            if (isSpinningCounterClockwise()
                    && spindexerCurrent > Constants.spindexerStuckCurrentMilliamps) {
                if (spindexerHighCurrentStartTime < 0) {
                    spindexerHighCurrentStartTime = now;
                }
            } else {
                spindexerHighCurrentStartTime = -1;
            }
            boolean spindexerCurrentJam = intaking && !shooting && intakeOn
                    && spindexerHighCurrentStartTime >= 0
                    && now - spindexerHighCurrentStartTime >= Constants.spindexerStuckDetectionTimeMs;

            // While shooting: a sustained jam-level current still reverses the intake
            // to clear it, but we do NOT touch the spindexer or the ball count.
            boolean shootingJam = shooting && intakeOn
                    && highCurrentDuration >= Constants.intakeStuckDetectionTimeMs;

            // One shared reverse per event (guarded by ignoreUnstuck) so the
            // detectors can never double-schedule the intake reverse.
            if ((intakeMotorJam || spindexerCurrentJam || shootingJam || fullJamReverse)
                    && !ignoreUnstuck) {
                intakeStuck = true;
                ignoreUnstuck = true;
                lastUnstuckTime = now;
                intakeHighCurrentStartTime = -1;
                spindexerHighCurrentStartTime = -1;

                if (intakeMotorJam && !spindexerCurrentJam) {
                    // Intake motor jam with the spindexer idle: reverse, then index
                    // a slot and count the freed ball.
                    jamIndexPending = true;
                    robot.intake.shortReverse()
                            .then(rotate120CCWAndIncrementCount())
                            .schedule();
                } else {
                    // Reverse the intake only - no spindexer turn, no count change.
                    // Covers: spindexer strain mid-CCW-turn, shooting, and a jam at
                    // 3 balls (spit the extra ball so it can't get stuck).
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
            telemetry.addData("Spindexer Offset Degrees",
                    manualOffsetTicks / Constants.spindexerTicksPerRevolution * 360.0);
            telemetry.addData("Spindexer Ignore Sensor", ignoreSensor);
            telemetry.addData("Spindexer Auto Load Pending", autoLoadPending);
            telemetry.addData("Spindexer Auto Index Armed", autoIndexArmed);
            telemetry.addData("Spindexer Intaking", intaking);
            telemetry.addData("Spindexer Shooting", shooting);
        });
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.concurrent.TimeUnit;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

@Config
public class Spindexer {
    public static String motorName = "spindexer";
    public static String rangerName = "ranger";

    public static double ticksPerRevolution = 537.7;
    public static double deadbandDegrees = 2.0;
    public static double movePower = 1.0;
    public static double shootPower = 0.7;
    public static double holdPower = 0.25;

    public static boolean autoSpindex = true;
    public static long sensorWaitMs = 250;
    public static long shootSensorWaitMs = 500;
    public static long shootSingleSensorWaitMs = 350;
    public static double stuckCurrentAmps = 8.0;
    public static long unstuckWaitMs = 750;

    private final DcMotorEx spindexerMotor;
    private final DigitalChannel ranger;
    private final Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();

    private double targetPositionTicks = 0;
    private double activePower = movePower;

    private boolean ignoreUnstuck = false;
    private long lastUnstuckTime = 0;

    private boolean canRotate = false;
    private boolean lastBallDetected = false;
    private boolean ballDetected = false;
    private boolean ignoreSensor = false;
    private long sensorWait = sensorWaitMs;
    private long lastDetectTime = 0;
    private int ballCount = 0;

    private boolean intaking = true;
    private boolean shooting = false;
    private boolean currentStuck = false;

    public Spindexer(Robot robot) {
        spindexerMotor = robot.hardwareMap.get(DcMotorEx.class, motorName);
        ranger = robot.hardwareMap.get(DigitalChannel.class, rangerName);
        telemetry = robot.telemetry;

        ranger.setMode(DigitalChannel.Mode.INPUT);

        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setTargetPosition(0);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexerMotor.setPower(holdPower);
    }

    public Command rotate360CW() {
        return instant(() -> {
            moveRelative(-ticksPerRevolution, shootPower);
            ballCount = 0;
        }).requiring(spindexerMotor);
    }

    public Command rotateShootCW() {
        return instant(() -> {
            intaking = false;
            shooting = true;
            moveRelative(-(ticksPerRevolution + ticks120Degrees()), shootPower);
            ignoreSensor(shootSensorWaitMs);
            ballCount = 0;
        }).requiring(spindexerMotor);
    }

    public Command rotateResetCW() {
        return instant(() -> {
            moveRelative(ticks120Degrees() - 20, shootPower);
            ballCount = 0;
        }).requiring(spindexerMotor);
    }

    public Command rotate360CCW() {
        return instant(() -> moveRelative(ticksPerRevolution, movePower)).requiring(spindexerMotor);
    }

    public Command rotate15CW() {
        return instant(() -> {
            moveRelative(ticksPerRevolution / 15.0, movePower);
            ballCount--;
        }).requiring(spindexerMotor);
    }

    public Command rotate120CCW() {
        return instant(() -> moveRelative(ticks120Degrees(), movePower)).requiring(spindexerMotor);
    }

    public Command rotate120CW() {
        return instant(() -> {
            moveRelative(-ticks120Degrees(), movePower);
            ignoreSensor(shootSingleSensorWaitMs);
            ballCount--;
        }).requiring(spindexerMotor);
    }

    public Command stop() {
        return instant(() -> {
            targetPositionTicks = spindexerMotor.getCurrentPosition();
            spindexerMotor.setTargetPosition(getTargetPosition());
            spindexerMotor.setPower(0);
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
            this.intaking = intaking;
            this.shooting = false;
        });
    }

    public boolean isMoving() {
        return Math.abs(targetPositionTicks - getCurrentPosition()) > deadbandTicks();
    }

    public int getCurrentPosition() {
        return spindexerMotor.getCurrentPosition();
    }

    public int getTargetPosition() {
        return (int) Math.round(targetPositionTicks);
    }

    public int getBallCount() {
        return ballCount;
    }

    public double getCurrent() {
        return spindexerMotor.getCurrent(CurrentUnit.AMPS);
    }

    private void moveRelative(double deltaTicks, double power) {
        targetPositionTicks += deltaTicks;
        activePower = Math.abs(power);
        spindexerMotor.setTargetPosition(getTargetPosition());
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexerMotor.setPower(activePower);
    }

    private void ignoreSensor(long waitMs) {
        ignoreSensor = true;
        sensorWait = waitMs;
        lastDetectTime = timer.time(TimeUnit.MILLISECONDS);
    }

    private double ticks120Degrees() {
        return ticksPerRevolution / 3.0;
    }

    private double deadbandTicks() {
        return (deadbandDegrees / 360.0) * ticksPerRevolution;
    }

    private void autoLoadBall() {
        ballCount++;
        if (ballCount < 3) {
            moveRelative(ticks120Degrees(), movePower);
        } else {
            intaking = false;
        }
    }

    public Command periodic() {
        return infinite(() -> {
            spindexerMotor.setTargetPosition(getTargetPosition());
            spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindexerMotor.setPower(isMoving() ? activePower : holdPower);

            canRotate = ranger.getState();
            ballDetected = canRotate;

            if (ballDetected && !lastBallDetected && autoSpindex && intaking && !ignoreSensor) {
                ignoreSensor(sensorWaitMs);
                autoLoadBall();
            }

            double current = getCurrent();
            long now = timer.time(TimeUnit.MILLISECONDS);
            if (current >= stuckCurrentAmps && !ignoreUnstuck) {
                currentStuck = true;
                ignoreUnstuck = true;
                lastUnstuckTime = now;
            }

            if (now - lastUnstuckTime >= unstuckWaitMs) {
                ignoreUnstuck = false;
                currentStuck = false;
            }

            if (now - lastDetectTime >= sensorWait) {
                ignoreSensor = false;
            }

            lastBallDetected = ballDetected;

            telemetry.addData("Spindexer Position", getCurrentPosition());
            telemetry.addData("Spindexer Target", getTargetPosition());
            telemetry.addData("Spindexer Moving", isMoving());
            telemetry.addData("Spindexer Current", current);
            telemetry.addData("Spindexer Stuck Current", currentStuck);
            telemetry.addData("Spindexer Ranger Can Rotate", canRotate);
            telemetry.addData("Spindexer Ball Count", ballCount);
            telemetry.addData("Spindexer Ignore Sensor", ignoreSensor);
            telemetry.addData("Spindexer Intaking", intaking);
            telemetry.addData("Spindexer Shooting", shooting);
        });
    }
}

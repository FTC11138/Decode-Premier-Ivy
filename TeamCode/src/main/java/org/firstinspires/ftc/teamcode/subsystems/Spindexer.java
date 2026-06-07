package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private final DigitalChannel ranger;
    private final Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();

    private double targetPositionTicks = 0;
    private double activePower = Constants.spindexerMovePower;

    private boolean ignoreUnstuck = false;
    private long lastUnstuckTime = 0;

    private boolean canRotate = false;
    private boolean lastBallDetected = false;
    private boolean ballDetected = false;
    private boolean ignoreSensor = false;
    private long sensorWait = Constants.sensorWait;
    private long lastDetectTime = 0;
    private int ballCount = 0;

    private boolean intaking = true;
    private boolean shooting = false;
    private boolean currentStuck = false;

    public Spindexer(Robot robot) {
        this.robot = robot;
        spindexerMotor = robot.hardwareMap.get(DcMotorEx.class, HardwareNames.spindexer);
        ranger = robot.hardwareMap.get(DigitalChannel.class, HardwareNames.ranger);
        telemetry = robot.telemetry;

        ranger.setMode(DigitalChannel.Mode.INPUT);

        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setTargetPosition(0);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexerMotor.setPower(Constants.spindexerHoldPower);
    }

    public Command rotate360CW() {
        return instant(() -> {
            moveRelative(-Constants.spindexerTicksPerRevolution, Constants.spindexerShootPower);
            ballCount = 0;
        }).requiring(spindexerMotor);
    }

    public Command rotateShootCW() {
        return instant(() -> {
            intaking = false;
            shooting = true;
            moveRelative(-(Constants.spindexerTicksPerRevolution + ticks120Degrees()), Constants.spindexerShootPower);
            ignoreSensor(Constants.shootSensorWait);
            ballCount = 0;
        }).requiring(spindexerMotor);
    }

    public Command rotateResetCW() {
        return instant(() -> {
            moveRelative(ticks120Degrees() - 20, Constants.spindexerShootPower);
            ballCount = 0;
        }).requiring(spindexerMotor);
    }

    public Command rotate360CCW() {
        return instant(() -> moveRelative(Constants.spindexerTicksPerRevolution, Constants.spindexerMovePower)).requiring(spindexerMotor);
    }

    public Command rotate15CW() {
        return instant(() -> {
            moveRelative(Constants.spindexerTicksPerRevolution / 15.0, Constants.spindexerMovePower);
            ballCount--;
        }).requiring(spindexerMotor);
    }

    public Command rotate120CCW() {
        return instant(() -> moveRelative(ticks120Degrees(), Constants.spindexerMovePower)).requiring(spindexerMotor);
    }

    public Command rotate120CW() {
        return instant(() -> {
            moveRelative(-ticks120Degrees(), Constants.spindexerMovePower);
            ignoreSensor(Constants.shootSingleSensorWait);
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
        return Constants.spindexerTicksPerRevolution / 3.0;
    }

    private double deadbandTicks() {
        return (Constants.spindexerDeadbandDegrees / 360.0) * Constants.spindexerTicksPerRevolution;
    }

    private void autoLoadBall() {
        ballCount++;
        if (ballCount < 3) {
            moveRelative(ticks120Degrees(), Constants.spindexerMovePower);
        } else {
            intaking = false;
        }
    }

    public Command periodic() {
        return infinite(() -> {
            spindexerMotor.setTargetPosition(getTargetPosition());
            spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindexerMotor.setPower(isMoving() ? activePower : Constants.spindexerHoldPower);

            canRotate = ranger.getState();
            ballDetected = canRotate;

            if (ballDetected && !lastBallDetected && Constants.autoSpindex && intaking && !ignoreSensor) {
                ignoreSensor(Constants.sensorWait);
                autoLoadBall();
            }

            double current = getCurrent();
            long now = timer.time(TimeUnit.MILLISECONDS);
            if (current >= Constants.stuckCurrent && !ignoreUnstuck) {
                currentStuck = true;
                ignoreUnstuck = true;
                lastUnstuckTime = now;
                robot.intake.shortReverse().schedule();
            }

            if (now - lastUnstuckTime >= Constants.spindexerUnstuckWaitMs) {
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

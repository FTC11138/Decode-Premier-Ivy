package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.TurretLocation;
import org.firstinspires.ftc.teamcode.math.WaveLength;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.HardwareNames;

import static com.pedropathing.ivy.commands.Commands.infinite;

public class Shooter {
    private final DcMotorEx flywheelMotorTop;
    private final DcMotorEx flywheelMotorBottom;
    private final Servo adjustableHood;
    private final Telemetry telemetry;
    private final Drivetrain drivetrain;
    private final VoltageSensor voltageSensor;
    private double cachedVoltage = Constants.shooterNominalVoltage;
    private long lastVoltageReadMs = 0;
    private double voltageScale = 1.0;
    private boolean tempOverride = false;
    private boolean hoodOverride = false;
    private boolean closeInterpolationOnly = false;
    private boolean farInterpolationOnly = false;
    private boolean on = false;
    private double target = 0;
    private double hoodTarget = Constants.adjHoodMin;
    // Modeled physical hood position. The servo gives no feedback, so we integrate
    // the commanded hood target at the servo's estimated slew rate to know when the
    // hood has ACTUALLY arrived (see updateHoodModel / atTarget).
    private double hoodModelPosition = Constants.adjHoodMin;
    private long hoodModelLastTime = System.nanoTime();
    // Settle-timeout bookkeeping: the hood goal value the settle clock is timing, and
    // when that clock started. atTarget() lets the hood count as ready once the model
    // arrives OR this clock expires, so a mis-estimated slew rate can never delay a
    // shot by more than Constants.shooterHoodMaxSettleMs.
    private double hoodSettleLastTarget = Constants.adjHoodMin;
    private long hoodSettleStartMs = System.currentTimeMillis();

    public Shooter(Robot robot) {
        flywheelMotorTop = robot.hardwareMap.get(DcMotorEx.class, HardwareNames.flywheelTop);
        flywheelMotorBottom = robot.hardwareMap.get(DcMotorEx.class, HardwareNames.flywheelBottom);
        adjustableHood = robot.hardwareMap.get(Servo.class, HardwareNames.adjustableHood);
        voltageSensor = robot.hardwareMap.voltageSensor.iterator().hasNext()
                ? robot.hardwareMap.voltageSensor.iterator().next()
                : null;
        initMotor(flywheelMotorTop);
        initMotor(flywheelMotorBottom);
        applyMotorDirections();

        telemetry = robot.telemetry;
        drivetrain = robot.drivetrain;
    }

    private void initMotor(DcMotorEx motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
    }

    private void applyMotorDirections() {
        flywheelMotorTop.setDirection(motorDirection(Constants.shooterTopReversed));
        flywheelMotorBottom.setDirection(motorDirection(Constants.shooterBottomReversed));
    }

    private DcMotorSimple.Direction motorDirection(boolean reversed) {
        return reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }

    public void setTarget(double target) {
        this.target = Math.abs(target);
        tempOverride = true;
        closeInterpolationOnly = false;
        farInterpolationOnly = false;
    }

    public void useInterpolation() {
        tempOverride = false;
        hoodOverride = false;
        closeInterpolationOnly = false;
        farInterpolationOnly = false;
    }

    public void useCloseInterpolation() {
        tempOverride = false;
        hoodOverride = false;
        closeInterpolationOnly = true;
        farInterpolationOnly = false;
    }

    public void useFarInterpolation() {
        tempOverride = false;
        hoodOverride = false;
        closeInterpolationOnly = false;
        farInterpolationOnly = true;
    }

    public void setHoodPosition(double hoodPosition) {
        hoodTarget = Range.clip(hoodPosition, Constants.adjHoodMax, Constants.adjHoodServoMax);
        hoodOverride = true;
    }

    private void setPower(double power) {
        flywheelMotorTop.setPower(Constants.shooterTopEnabled ? power : 0);
        flywheelMotorBottom.setPower(Constants.shooterBottomEnabled ? power : 0);
    }

    private void setVelocity(double tps) {
        flywheelMotorTop.setVelocity(tps);
        flywheelMotorBottom.setVelocity(tps);
    }

    private double flywheelIntegral = 0;
    private double flywheelLastError = 0;
    private long flywheelLastTime = System.nanoTime();
    public double flywheelPIDF(
            double targetVelocity,
            double currentVelocity,
            double kP,
            double kI,
            double kD,
            double kV,
            double kS,
            double voltageScale
    ) {
        long now = System.nanoTime();
        double dt = (now - flywheelLastTime) / 1e9;

        if (dt <= 0 || dt > 0.25) {
            dt = 0.02;
        }
        double error = targetVelocity - currentVelocity;

        // Integral-zone guard: only accumulate when close to target so the
        // integral does not wind up during the large-error spin-up phase.
        if (Math.abs(error) <= Constants.shooterIntegralZoneTps) {
            flywheelIntegral += error * dt;
            flywheelIntegral = Math.max(-5000, Math.min(5000, flywheelIntegral));
        }

        double derivative = (error - flywheelLastError) / dt;

        flywheelLastError = error;
        flywheelLastTime = now;

        double pid = (kP * error) + (kI * flywheelIntegral) + (kD * derivative);
        double feedforward = (kV * targetVelocity) + (kS * Math.signum(targetVelocity));
        // Voltage compensation: scale the whole command so the delivered motor
        // voltage stays constant regardless of battery sag.
        double output = (pid + feedforward) * voltageScale;
        return Math.max(-1, Math.min(1, output));
    }

    private double getVelocity() {
        return Math.abs(flywheelMotorBottom.getVelocity());
    }

    // Advance the modeled hood position toward the commanded target at the servo's
    // estimated slew rate. The hood servo has no feedback, so getPosition() only
    // echoes the last command; this model is how atTarget() knows the hood has
    // PHYSICALLY arrived instead of merely having been commanded this loop.
    private void updateHoodModel(double commanded) {
        long now = System.nanoTime();
        double dt = (now - hoodModelLastTime) / 1e9;
        hoodModelLastTime = now;
        if (dt <= 0 || dt > 0.25) {
            dt = 0.02;
        }
        double maxStep = Constants.shooterHoodSlewRatePerSecond * dt;
        double diff = commanded - hoodModelPosition;
        if (Math.abs(diff) <= maxStep) {
            hoodModelPosition = commanded;
        } else {
            hoodModelPosition += Math.signum(diff) * maxStep;
        }
    }

    public boolean atTarget() {
        boolean velocityReady = Math.abs(target - getVelocity()) <= Constants.shooterVelocityTolerance;
        boolean hoodReady;
        if (Constants.shooterHoodModelEnabled) {
            // Ready when the model has arrived, OR the bounded settle timeout has
            // expired. The timeout guarantees a wrong slew-rate estimate can never
            // delay a shot more than shooterHoodMaxSettleMs (protects the un-timed
            // TeleOp trigger); worst case this degrades to master's behavior.
            boolean hoodModelReady = Math.abs(hoodModelPosition - hoodTarget) <= Constants.shooterHoodTolerance;
            boolean settleTimedOut =
                    (System.currentTimeMillis() - hoodSettleStartMs) >= Constants.shooterHoodMaxSettleMs;
            hoodReady = hoodModelReady || settleTimedOut;
        } else {
            // Fallback to master behavior: the servo only echoes the last command,
            // so this effectively does not gate on the hood settling at all.
            hoodReady = Math.abs(adjustableHood.getPosition() - hoodTarget) <= Constants.shooterHoodTolerance;
        }
        return on && target != 0 && velocityReady && hoodReady;
    }

    public boolean isOn() {
        return on;
    }

    public void turnOn() {
        on = true;
    }

    public void turnOff() {
        on = false;
        flywheelIntegral = 0;
        flywheelLastError = 0;
    }

    public void toggle() {
        on = !on;
        if (on) turnOn();
        else turnOff();
    }

    public Command periodic() {
        return infinite(() -> {
            com.pedropathing.geometry.Pose turretPose = TurretLocation.getTurretPose(drivetrain.getPose());
            double goalDistance = WaveLength.getDistanceToGoal(turretPose, Alliance.current);
            applyMotorDirections();

            // Refresh battery voltage at ~10 Hz only; the reading is a hub
            // round-trip (~2-3 ms, not bulk-cached) and voltage moves slowly.
            long nowMs = System.currentTimeMillis();
            if (voltageSensor != null && nowMs - lastVoltageReadMs >= 100) {
                cachedVoltage = voltageSensor.getVoltage();
                lastVoltageReadMs = nowMs;
            }
            voltageScale = 1.0;
            if (Constants.shooterVoltageComp && cachedVoltage > 6.0) {
                voltageScale = Range.clip(Constants.shooterNominalVoltage / cachedVoltage, 1.0, 1.5);
            }

            if (on) {
                target = Constants.shooterOverride
                        ? Math.abs(Constants.shooterOverrideTarget)
                        : (tempOverride
                        ? target
                        : (closeInterpolationOnly
                        ? WaveLength.getCloseVelocityWithInterpolation(turretPose, Alliance.current)
                        : (farInterpolationOnly
                        ? WaveLength.getFarVelocityWithInterpolation(turretPose, Alliance.current)
                        : WaveLength.getVelocityWithInterpolation(turretPose, Alliance.current))));
                hoodTarget = hoodOverride
                        ? hoodTarget
                        : (closeInterpolationOnly
                        ? WaveLength.getCloseHoodWithInterpolation(turretPose, Alliance.current)
                        : (farInterpolationOnly
                        ? WaveLength.getFarHoodWithInterpolation(turretPose, Alliance.current)
                        : WaveLength.getHoodWithInterpolation(turretPose, Alliance.current)));
                double pidPower = flywheelPIDF(
                        target,
                        getVelocity(),
                        Constants.shooterKp,
                        Constants.shooterKi,
                        Constants.shooterKd,
                        Constants.shooterKv,
                        Constants.shooterKs,
                        voltageScale
                );
                setPower(Range.clip(Math.signum(Constants.shooterPowerSign) * Math.max(0, pidPower), -1, 1));
            } else {
                target = 0;
                hoodTarget = Constants.adjHoodMin;
                setPower(0);
            }

            double hoodCommand = Range.clip(
                    hoodTarget,
                    Constants.adjHoodMax,
                    Constants.adjHoodServoMax
            );
            adjustableHood.setPosition(hoodCommand);
            updateHoodModel(hoodCommand);

            // Restart the settle clock whenever the hood goal moves meaningfully.
            if (Math.abs(hoodTarget - hoodSettleLastTarget) > Constants.shooterHoodTolerance) {
                hoodSettleLastTarget = hoodTarget;
                hoodSettleStartMs = System.currentTimeMillis();
            }
            boolean hoodSettling = Constants.shooterHoodModelEnabled
                    && Math.abs(hoodModelPosition - hoodTarget) > Constants.shooterHoodTolerance
                    && (System.currentTimeMillis() - hoodSettleStartMs) < Constants.shooterHoodMaxSettleMs;

            telemetry.addData("Shooter Distance", goalDistance);
            telemetry.addData("Shooter On", on);
            telemetry.addData("Shooter Top Enabled", Constants.shooterTopEnabled);
            telemetry.addData("Shooter Bottom Enabled", Constants.shooterBottomEnabled);
            telemetry.addData("Flywheel Velocity", getVelocity());
            telemetry.addData("Flywheel Target", target);
            telemetry.addData("Flywheel Top Power", flywheelMotorTop.getPower());
            telemetry.addData("Flywheel Bottom Power", flywheelMotorBottom.getPower());
            telemetry.addData("Hood Target", hoodTarget);
            telemetry.addData("Hood Position", adjustableHood.getPosition());
            telemetry.addData("Hood Model Pos", hoodModelPosition);
            telemetry.addData("Hood Settling", hoodSettling);
            telemetry.addData("Flywheel VoltScale", voltageScale);
            telemetry.addData("Battery V", cachedVoltage);
            telemetry.addData("Close Interpolation Only", closeInterpolationOnly);
            telemetry.addData("Far Interpolation Only", farInterpolationOnly);
        });
    }
}

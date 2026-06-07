package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private boolean tempOverride = false;
    private boolean hoodOverride = false;
    private boolean on = false;
    private double target = 0;
    private double hoodTarget = Constants.adjHoodMin;

    public Shooter(Robot robot) {
        flywheelMotorTop = robot.hardwareMap.get(DcMotorEx.class, HardwareNames.flywheelTop);
        flywheelMotorBottom = robot.hardwareMap.get(DcMotorEx.class, HardwareNames.flywheelBottom);
        adjustableHood = robot.hardwareMap.get(Servo.class, HardwareNames.adjustableHood);
        flywheelMotorBottom.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = robot.telemetry;
        drivetrain = robot.drivetrain;
    }

    public void setTarget(double target) {
        this.target = Math.abs(target);
        tempOverride = true;
    }

    public void useInterpolation() {
        tempOverride = false;
        hoodOverride = false;
    }

    public void setHoodPosition(double hoodPosition) {
        hoodTarget = Range.clip(hoodPosition, Constants.adjHoodMax, Constants.adjHoodMin);
        hoodOverride = true;
    }

    private void setPower(double power) {
        flywheelMotorTop.setPower(power);
        flywheelMotorBottom.setPower(power);
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
            double kS
    ) {
        long now = System.nanoTime();
        double dt = (now - flywheelLastTime) / 1e9;

        if (dt <= 0 || dt > 0.25) {
            dt = 0.02;
        }
        double error = targetVelocity - currentVelocity;

        flywheelIntegral += error * dt;
        flywheelIntegral = Math.max(-5000, Math.min(5000, flywheelIntegral));

        double derivative = (error - flywheelLastError) / dt;

        flywheelLastError = error;
        flywheelLastTime = now;

        double pid = (kP * error) + (kI * flywheelIntegral) + (kD * derivative);
        double feedforward = (kV * targetVelocity) + (kS * Math.signum(targetVelocity));
        return Math.max(-1, Math.min(1, pid + feedforward));
    }

    private double getVelocity() {
        return Math.abs(flywheelMotorBottom.getVelocity());
    }

    public boolean atTarget() {
        boolean velocityReady = Math.abs(target - getVelocity()) <= Constants.shooterVelocityTolerance;
        boolean hoodReady = Math.abs(adjustableHood.getPosition() - hoodTarget) <= Constants.shooterHoodTolerance;
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

            if (on) {
                target = Constants.shooterOverride ? Math.abs(Constants.shooterOverrideTarget) : (tempOverride ? target : WaveLength.getVelocityWithInterpolation(turretPose, Alliance.current));
                hoodTarget = hoodOverride ? hoodTarget : WaveLength.getHoodWithInterpolation(turretPose, Alliance.current);
                setPower(flywheelPIDF(
                        target,
                        getVelocity(),
                        Constants.shooterKp,
                        Constants.shooterKi,
                        Constants.shooterKd,
                        Constants.shooterKv,
                        Constants.shooterKs
                ));
            } else {
                target = 0;
                hoodTarget = Constants.adjHoodMin;
                setPower(0);
            }

            adjustableHood.setPosition(Range.clip(hoodTarget, Constants.adjHoodMax, Constants.adjHoodMin));

            telemetry.addData("Shooter Distance", goalDistance);
            telemetry.addData("Flywheel Velocity", getVelocity());
            telemetry.addData("Flywheel Target", target);
            telemetry.addData("Flywheel Power", flywheelMotorBottom.getPower());
            telemetry.addData("Hood Target", hoodTarget);
            telemetry.addData("Hood Position", adjustableHood.getPosition());
        });
    }
}

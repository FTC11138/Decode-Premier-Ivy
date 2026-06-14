package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.HardwareNames;

import static com.pedropathing.ivy.commands.Commands.infinite;

public class Turret {
    private static double angleTransfer = 0;
    private final CRServo turretServo;
    private final DcMotorEx turretEncoder;
    private final Telemetry telemetry;
    public boolean forceOff = false;
    public boolean autoAimEnabled = true;
    private double targetDegrees = 0;
    private Mode mode = Mode.OFF;
    private double angleOffsetDegrees = 0;
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = System.nanoTime();
    private double lastPower = 0;

    public Turret(Robot robot) {
        turretServo = robot.hardwareMap.get(CRServo.class, HardwareNames.turretServo);
        turretEncoder = robot.hardwareMap.get(DcMotorEx.class, HardwareNames.turretEncoder);
        telemetry = robot.telemetry;

        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double getRawAngleDegrees() {
        double direction = Constants.turretEncoderReversed ? -1.0 : 1.0;
        return direction * turretEncoder.getCurrentPosition() / Constants.turretTicksPerRevolution * 360;
    }

    private double getAngleDegrees() {
        return getRawAngleDegrees() + angleOffsetDegrees;
    }

    private void setAngleDegrees(double angle) {
        angleOffsetDegrees = angle - getRawAngleDegrees();
    }

    public static void localize(double angle) {
        angleTransfer = angle;
    }

    public void off() {
        if (mode == Mode.HOME) return;
        mode = Mode.OFF;
        resetController();
    }

    public double getTargetDegrees() {
        return targetDegrees;
    }

    public void setTargetDegrees(double targetDegrees) {
        if (mode == Mode.HOME) return;
        mode = Mode.POSITION;
        this.targetDegrees = Range.clip(targetDegrees, Constants.turretMinAngleDegrees, Constants.turretMaxAngleDegrees);
    }

    public void home() {
        setAngleDegrees(Constants.turretHomedAngleDegrees);
        targetDegrees = Range.clip(Constants.turretHomedAngleDegrees, Constants.turretMinAngleDegrees, Constants.turretMaxAngleDegrees);
        mode = Mode.POSITION;
        resetController();
    }

    public void moveLeft() {
        autoAimEnabled = false;
        setTargetDegrees(targetDegrees - Constants.turretIncrementDegrees);
    }

    public void moveRight() {
        autoAimEnabled = false;
        setTargetDegrees(targetDegrees + Constants.turretIncrementDegrees);
    }

    public void enableAutoAim() {
        autoAimEnabled = true;
    }

    public void disableAutoAim() {
        autoAimEnabled = false;
    }

    public void setStartingAngle(double angle) {
        setAngleDegrees(angle);
    }

    public void usePreviousStartingAngle() {
        setAngleDegrees(angleTransfer);
    }

    private void resetController() {
        integral = 0;
        lastError = 0;
        lastTime = System.nanoTime();
        lastPower = 0;
    }

    private void setTurretPower(double power) {
        double angle = getAngleDegrees();
        if (angle <= Constants.turretMinAngleDegrees && power < 0) {
            power = 0;
        } else if (angle >= Constants.turretMaxAngleDegrees && power > 0) {
            power = 0;
        }

        double direction = Constants.turretServoReversed ? -1.0 : 1.0;
        lastPower = Range.clip(direction * power, -Constants.maxPower, Constants.maxPower);
        turretServo.setPower(lastPower);
    }

    private double calculatePower() {
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        if (dt <= 0 || dt > 0.25) dt = 0.02;

        double error = targetDegrees - getAngleDegrees();
        if (Math.abs(error) <= Constants.deadbandDeg) {
            resetController();
            return 0;
        }

        integral += error * dt;
        integral = Range.clip(integral, -Constants.maxIntegral, Constants.maxIntegral);

        double derivative = Range.clip((error - lastError) / dt, -Constants.maxDeriv, Constants.maxDeriv);
        double power = Constants.kP_v * error + Constants.kI_v * integral + Constants.kD_v * derivative;

        if (Constants.kS != 0) {
            power += Math.signum(power) * Constants.kS;
        }

        lastError = error;
        lastTime = now;
        return Range.clip(power, -Constants.maxPower, Constants.maxPower);
    }

    public Command periodic() {
        return infinite(() -> {
            if (forceOff) {
                setTurretPower(0);
                resetController();
            }
            else {
                switch (mode) {
                    case POSITION:
                        setTurretPower(calculatePower());
                        break;
                    case OFF:
                        setTurretPower(0);
                        resetController();
                        break;
                    case HOME:
                        setAngleDegrees(Constants.turretHomedAngleDegrees);
                        setTargetDegrees(Constants.turretHomedAngleDegrees);
                        break;
                }
            }

            angleTransfer = getAngleDegrees();

            telemetry.addData("Turret Angle", getAngleDegrees());
            telemetry.addData("Turret Target", targetDegrees);
            telemetry.addData("Turret Error", targetDegrees - getAngleDegrees());
            telemetry.addData("Turret CR Power", lastPower);
            telemetry.addData("Turret Mode", mode);
            telemetry.addData("Turret Auto Aim", autoAimEnabled);
            telemetry.addData("Turret Encoder Ticks", turretEncoder.getCurrentPosition());
            telemetry.addData("Turret Forced Off", forceOff ? "Yes" : "No");
        });
    }

    public enum Mode {
        POSITION,
        OFF,
        HOME
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static com.pedropathing.ivy.commands.Commands.infinite;

@Config
public class Turret {
    private static final double TICKS_PER_REVOLUTION = 384.5 * 3;
    private static double angleTransfer = 0;
    public static double incrementDegrees = 2.5;
    public static double homedAngleDegrees = 145;
    public static double minAngleDegrees = -90;
    public static double maxAngleDegrees = 270;
    public static double minServoPosition = 0.0;
    public static double maxServoPosition = 1.0;
    public static double disabledServoPosition = 0.0;
    private final Servo turretServo;
    private final DcMotorEx turretEncoder;
    private final Telemetry telemetry;
    public boolean forceOff = false;
    private double targetDegrees = 0;
    private Mode mode = Mode.OFF;
    private double angleOffsetDegrees = 0;

    public Turret(Robot robot) {
        turretServo = robot.hardwareMap.get(Servo.class, "turret");
        turretEncoder = robot.hardwareMap.get(DcMotorEx.class, "turretEncoder");
        telemetry = robot.telemetry;

        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double getRawAngleDegrees() {
        return turretEncoder.getCurrentPosition() / TICKS_PER_REVOLUTION * 360;
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
    }

    public double getTargetDegrees() {
        return targetDegrees;
    }

    public void setTargetDegrees(double targetDegrees) {
        if (mode == Mode.HOME) return;
        mode = Mode.POSITION;
        this.targetDegrees = Range.clip(targetDegrees, minAngleDegrees, maxAngleDegrees);
    }

    public void home() {
        setAngleDegrees(homedAngleDegrees);
        setTargetDegrees(homedAngleDegrees);
    }

    public void moveLeft() {
        setTargetDegrees(targetDegrees - incrementDegrees);
    }

    public void moveRight() {
        setTargetDegrees(targetDegrees + incrementDegrees);
    }

    public void setStartingAngle(double angle) {
        setAngleDegrees(angle);
    }

    public void usePreviousStartingAngle() {
        setAngleDegrees(angleTransfer);
    }

    private double angleToServoPosition(double angleDegrees) {
        double clippedAngle = Range.clip(angleDegrees, minAngleDegrees, maxAngleDegrees);
        double angleRange = maxAngleDegrees - minAngleDegrees;
        if (angleRange == 0) return minServoPosition;

        double percent = (clippedAngle - minAngleDegrees) / angleRange;
        double position = minServoPosition + percent * (maxServoPosition - minServoPosition);
        return Range.clip(position, 0.0, 1.0);
    }

    public Command periodic() {
        return infinite(() -> {
            if (forceOff) turretServo.setPosition(disabledServoPosition);
            else {
                switch (mode) {
                    case POSITION:
                        turretServo.setPosition(angleToServoPosition(targetDegrees));
                        break;
                    case OFF:
                        break;
                    case HOME:
                        setAngleDegrees(homedAngleDegrees);
                        setTargetDegrees(homedAngleDegrees);
                        break;
                }
            }

            angleTransfer = getAngleDegrees();

            telemetry.addData("Turret Angle", getAngleDegrees());
            telemetry.addData("Turret Target", targetDegrees);
            telemetry.addData("Turret Servo Position", turretServo.getPosition());
            telemetry.addData("Turret Target Servo Position", angleToServoPosition(targetDegrees));
            telemetry.addData("Turret Mode", mode);
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

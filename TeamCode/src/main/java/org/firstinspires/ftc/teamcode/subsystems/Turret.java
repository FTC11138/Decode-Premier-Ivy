package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.HardwareNames;

import static com.pedropathing.ivy.commands.Commands.infinite;

public class Turret {
    private static double angleTransfer = 0;
    private final Servo turretServo;
    private final DcMotorEx turretEncoder;
    private final Telemetry telemetry;
    public boolean forceOff = false;
    private double targetDegrees = 0;
    private Mode mode = Mode.OFF;
    private double angleOffsetDegrees = 0;

    public Turret(Robot robot) {
        turretServo = robot.hardwareMap.get(Servo.class, HardwareNames.turretServo);
        turretEncoder = robot.hardwareMap.get(DcMotorEx.class, HardwareNames.turretEncoder);
        telemetry = robot.telemetry;

        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double getRawAngleDegrees() {
        return turretEncoder.getCurrentPosition() / Constants.turretTicksPerRevolution * 360;
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
        this.targetDegrees = Range.clip(targetDegrees, Constants.turretMinAngleDegrees, Constants.turretMaxAngleDegrees);
    }

    public void home() {
        setAngleDegrees(Constants.turretHomedAngleDegrees);
        setTargetDegrees(Constants.turretHomedAngleDegrees);
    }

    public void moveLeft() {
        setTargetDegrees(targetDegrees - Constants.turretIncrementDegrees);
    }

    public void moveRight() {
        setTargetDegrees(targetDegrees + Constants.turretIncrementDegrees);
    }

    public void setStartingAngle(double angle) {
        setAngleDegrees(angle);
    }

    public void usePreviousStartingAngle() {
        setAngleDegrees(angleTransfer);
    }

    private double angleToServoPosition(double angleDegrees) {
        double clippedAngle = Range.clip(angleDegrees, Constants.turretMinAngleDegrees, Constants.turretMaxAngleDegrees);
        double angleRange = Constants.turretMaxAngleDegrees - Constants.turretMinAngleDegrees;
        if (angleRange == 0) return Constants.turretMinServoPosition;

        double percent = (clippedAngle - Constants.turretMinAngleDegrees) / angleRange;
        double position = Constants.turretMinServoPosition + percent * (Constants.turretMaxServoPosition - Constants.turretMinServoPosition);
        return Range.clip(position, 0.0, 1.0);
    }

    public Command periodic() {
        return infinite(() -> {
            if (forceOff) turretServo.setPosition(Constants.turretDisabledServoPosition);
            else {
                switch (mode) {
                    case POSITION:
                        turretServo.setPosition(angleToServoPosition(targetDegrees));
                        break;
                    case OFF:
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

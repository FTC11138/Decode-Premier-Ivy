package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.HardwareNames;

import static com.pedropathing.ivy.commands.Commands.*;

public class Intake {
    private boolean slowMode = false;
    private Mode mode = Mode.OFF;

    private final Robot robot;
    private final DcMotorEx intakeMotor;

    private final CRServo intakeServo;
    private final Telemetry telemetry;

    public Intake(Robot robot) {
        this.robot = robot;
        intakeServo = robot.hardwareMap.get(CRServo.class, HardwareNames.intakeServo);
        intakeMotor = robot.hardwareMap.get(DcMotorEx.class, HardwareNames.intake);
        telemetry = robot.telemetry;
    }

    public Command on() {
        return instant(() -> mode = Mode.ON).requiring(intakeMotor);
    }
    public Command off() {
        return instant(() -> mode = Mode.OFF).requiring(intakeMotor);
    }

    public Command reverse() {
        return instant(() -> mode = Mode.REVERSE).requiring(intakeMotor);
    }

    public Command shortReverse() {
        return reverse().then(waitMs(Constants.intakeShortReverseTimeMs)).then(on());
    }

    public Command toggle() {
        return conditional(() -> mode == Mode.OFF, on(), off());
    }

    public double getIntakeCurrent() {return intakeMotor.getCurrent(CurrentUnit.MILLIAMPS);}

    public boolean isOn() {
        return mode == Mode.ON;
    }

    public void slowDown() {
        slowMode = true;
    }

    public void speedUp() {`
        slowMode = false;
    }

    public Command periodic() {
        return infinite(() -> {
            switch (mode) {
                case ON:
                    intakeMotor.setPower(slowMode ? Constants.intakeSlowPowerClose : Constants.intakeFastPower);
                    intakeServo.setPower(-1);
                    break;
                case OFF:
                    intakeMotor.setPower(Constants.intakeOffPower);
                    intakeServo.setPower(0);
                    break;
                case REVERSE:
                    intakeMotor.setPower(Constants.intakeReversePower);
                    intakeServo.setPower(0);
                    break;
            }

            // While the spindexer indexes counterclockwise (not the shooting
            // direction), drive the intake servo forward to help feed balls.
            boolean spindexerCounterClockwise = robot.spindexer.isSpinningCounterClockwise();
            if (spindexerCounterClockwise) {
                intakeServo.setPower(1);
            }

            telemetry.addData("Intake Servo Feed (Spindexer CCW)", spindexerCounterClockwise);
            telemetry.addData("Intake Current", intakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Intake Velocity", intakeMotor.getVelocity());
        });
    }

    enum Mode {
        ON,
        OFF,
        REVERSE
    }
}

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
    private long lastSpindexerCcwMs = 0;

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

    public Command reverseSlow() {
        return instant(() -> mode = Mode.REVERSE_SLOW).requiring(intakeMotor);
    }

    // Jam nudge: gentle reverse so it doesn't spit loaded balls out of the robot.
    public Command shortReverse() {
        return reverseSlow().then(waitMs(Constants.intakeShortReverseTimeMs)).then(on());
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

    public void speedUp() {
        slowMode = false;
    }

    public Command periodic() {
        return infinite(() -> {
            switch (mode) {
                case ON:
                    intakeMotor.setPower(slowMode ? Constants.intakeSlowPowerClose : Constants.intakeFastPower);
                    break;
                case OFF:
                    intakeMotor.setPower(Constants.intakeOffPower);
                    break;
                case REVERSE:
                    intakeMotor.setPower(Constants.intakeReversePower);
                    break;
                case REVERSE_SLOW:
                    intakeMotor.setPower(Constants.intakeJamReversePower);
                    break;
            }

            // The feed servo always tracks the spindexer:
            //  - spindexer counterclockwise -> +1 (outwards), overrides everything,
            //    and keeps going intakeServoCcwCoastMs after the turn ends
            //  - otherwise intake on OR spindexer clockwise -> -1 (inwards)
            //  - shooting (without a turn) or nothing happening -> 0
            // Intake-on / clockwise / shooting all supersede the CCW coast.
            boolean ccw = robot.spindexer.isSpinningCounterClockwise();
            boolean cw = robot.spindexer.isSpinningClockwise();
            boolean shooting = robot.spindexer.isShooting();
            boolean intakeOn = mode == Mode.ON;

            if (ccw) {
                lastSpindexerCcwMs = System.currentTimeMillis();
            }
            boolean ccwCoast =
                    System.currentTimeMillis() - lastSpindexerCcwMs < Constants.intakeServoCcwCoastMs;

            double servoPower;
            if (ccw) {
                servoPower = 1;
            } else if (intakeOn || cw || shooting) {
                servoPower = (intakeOn || cw) ? -1 : 0;
            } else if (ccwCoast) {
                servoPower = 1;
            } else {
                servoPower = 0;
            }
            intakeServo.setPower(servoPower);

            telemetry.addData("Intake Servo Power", servoPower);
            telemetry.addData("Intake Current", intakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Intake Velocity", intakeMotor.getVelocity());
        });
    }

    enum Mode {
        ON,
        OFF,
        REVERSE,
        REVERSE_SLOW
    }
}

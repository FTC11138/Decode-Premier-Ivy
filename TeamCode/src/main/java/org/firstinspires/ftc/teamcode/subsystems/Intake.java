package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static com.pedropathing.ivy.commands.Commands.*;

@Config
public class Intake {
    private boolean slowMode = false;
    private Mode mode = Mode.OFF;
    public static double fastPower = -1;
    public static double slowPower = -1;
    public static double offPower = 0;
    public static double reversePower = 1;
    public static double shortReverseTimeMs = 150;

    private final DcMotorEx intakeMotor;

    private final Telemetry telemetry;

    public Intake(Robot robot) {
        intakeMotor = robot.hardwareMap.get(DcMotorEx.class, "intake");
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
        return reverse().then(waitMs(shortReverseTimeMs)).then(on());
    }

    public Command toggle() {
        return conditional(() -> mode == Mode.OFF, on(), off());
    }

    public double getIntakeCurrent() {return intakeMotor.getCurrent(CurrentUnit.MILLIAMPS);}

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
                    intakeMotor.setPower(slowMode ? slowPower : fastPower);
                    break;
                case OFF:
                    intakeMotor.setPower(offPower);
                    break;
                case REVERSE:
                    intakeMotor.setPower(reversePower);
                    break;
            }

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
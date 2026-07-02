package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.HardwareNames;

import static com.pedropathing.ivy.commands.Commands.infinite;

/**
 * PWM RGB indicator lights (Base10 3118-0808-0002), driven as servos.
 *
 * led1: solid purple while the shooter velocity and hood are both on target,
 *       off otherwise.
 * led2: reflects the current spindexer ball count -
 *       0 = red, 1 = orange, 2 = blue, 3 = green.
 */
public class Leds {
    private final Robot robot;
    private final Servo led1;
    private final Servo led2;
    private final Telemetry telemetry;

    public Leds(Robot robot) {
        this.robot = robot;
        led1 = robot.hardwareMap.get(Servo.class, HardwareNames.led1);
        led2 = robot.hardwareMap.get(Servo.class, HardwareNames.led2);
        telemetry = robot.telemetry;

        // The color chart maps position 0-1 to 500-2500us pulses. Force that
        // full PWM range so the FTC values land on the intended colors.
        setFullPwmRange(led1);
        setFullPwmRange(led2);
    }

    private void setFullPwmRange(Servo servo) {
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
    }

    private double ballCountColor(int ballCount) {
        switch (Range.clip(ballCount, 0, 3)) {
            case 0:
                return Constants.ledRed;
            case 1:
                return Constants.ledOrange;
            case 2:
                return Constants.ledBlue;
            case 3:
                return Constants.ledGreen;
            default:
                return Constants.ledOff;
        }
    }

    public Command periodic() {
        return infinite(() -> {
            boolean shooterReady = robot.shooter.atTarget();
            int ballCount = robot.spindexer.getBallCount();

            double led1Color = shooterReady ? Constants.ledPurple : Constants.ledOff;
            double led2Color = ballCountColor(ballCount);

            led1.setPosition(led1Color);
            led2.setPosition(led2Color);

            telemetry.addData("LED1 Shooter Ready", shooterReady);
            telemetry.addData("LED1 Position", led1Color);
            telemetry.addData("LED2 Ball Count", ballCount);
            telemetry.addData("LED2 Position", led2Color);
        });
    }
}

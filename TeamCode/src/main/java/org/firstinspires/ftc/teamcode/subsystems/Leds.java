package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

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
        // led1 (the "ready" light) is unwired for now; tolerate it being absent
        // from the config so the working ball-count light still runs.
        led1 = tryGetServo(robot, HardwareNames.led1);
        led2 = robot.hardwareMap.get(Servo.class, HardwareNames.led2);
        telemetry = robot.telemetry;

        // The Base10 chart's FTC values assume the standard 600-2400us servo
        // range. Pin that explicitly so each value lands on the intended color.
        // (Forcing 500-2500 pushed red down to ~1054us, below the panel's 1100us
        // "off" threshold, which is why red read as off.)
        setChartPwmRange(led1);
        setChartPwmRange(led2);
    }

    private Servo tryGetServo(Robot robot, String name) {
        try {
            return robot.hardwareMap.get(Servo.class, name);
        } catch (RuntimeException e) {
            return null;
        }
    }

    private void setChartPwmRange(Servo servo) {
        if (servo instanceof ServoImplEx) {
            ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(600, 2400));
        }
    }

    private double ballCountColor(int ballCount) {
        switch (ballCount) {
            case 1:
                return Constants.ledOrange;
            case 2:
                return Constants.ledBlue;
            case 3:
                return Constants.ledGreen;
            default:
                // 0 (and any other value) stays red.
                return Constants.ledRed;
        }
    }

    public Command periodic() {
        return infinite(() -> {
            boolean shooterReady = robot.shooter.atTarget();
            int ballCount = robot.spindexer.getBallCount();

            double led1Color = shooterReady ? Constants.ledPurple : Constants.ledOff;
            // The dedicated "ready" LED (led1) is unwired for now, so surface the
            // ready state on the working ball-count LED:
            //   3 balls + ready -> purple
            //   2 balls + ready -> ledReadyTwoBalls (distinct, near-purple)
            // (2 is a ready state because the 3rd ball sometimes doesn't register.)
            // When not ready it shows the count color, so blue vs green still tells
            // you 2 vs 3 before the shooter spins up.
            double led2Color;
            if (shooterReady && ballCount >= 3) {
                led2Color = Constants.ledPurple;
            } else if (shooterReady && ballCount == 2) {
                led2Color = Constants.ledReadyTwoBalls;
            } else {
                led2Color = ballCountColor(ballCount);
            }

            if (led1 != null) {
                led1.setPosition(led1Color);
            }
            led2.setPosition(led2Color);

            telemetry.addData("LED1 Shooter Ready", shooterReady);
            telemetry.addData("LED1 Position", led1Color);
            telemetry.addData("LED2 Ball Count", ballCount);
            telemetry.addData("LED2 Position", led2Color);
        });
    }
}

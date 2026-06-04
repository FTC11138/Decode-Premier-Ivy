package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class Robot {
    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;
    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Shooter shooter;
    public final Turret turret;
    public final Spindexer spindexer;

    public Robot(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = new MultipleTelemetry(
                opMode.telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );

        drivetrain = new Drivetrain(this);
        intake = new Intake(this);
        shooter = new Shooter(this);
        turret = new Turret(this);
        spindexer = new Spindexer(this);
    }
}

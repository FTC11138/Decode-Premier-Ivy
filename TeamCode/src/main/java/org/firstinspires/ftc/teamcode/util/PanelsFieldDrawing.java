package org.firstinspires.ftc.teamcode.util;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public final class PanelsFieldDrawing {
    private static final double ROBOT_RADIUS_INCHES = 9.0;
    private static final FieldManager field = PanelsField.INSTANCE.getField();
    private static final Style robotStyle = new Style("", "#4CAF50", 0.8);

    private PanelsFieldDrawing() {
    }

    public static void init() {
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    public static void drawRobot(Pose pose) {
        if (pose == null
                || Double.isNaN(pose.getX())
                || Double.isNaN(pose.getY())
                || Double.isNaN(pose.getHeading())) {
            return;
        }

        field.setStyle(robotStyle);
        field.moveCursor(pose.getX(), pose.getY());
        field.circle(ROBOT_RADIUS_INCHES);

        Vector heading = pose.getHeadingAsUnitVector();
        heading.setMagnitude(ROBOT_RADIUS_INCHES);
        field.moveCursor(
                pose.getX() + heading.getXComponent() / 2.0,
                pose.getY() + heading.getYComponent() / 2.0
        );
        field.line(
                pose.getX() + heading.getXComponent(),
                pose.getY() + heading.getYComponent()
        );
        field.update();
    }
}

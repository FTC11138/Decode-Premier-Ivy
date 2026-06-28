package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.robot.Alliance;

import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

import static org.firstinspires.ftc.teamcode.math.PoseMirror.mirror;

public class WaveLength {
    // X = robot's left-right position on field (inches)
    // Y = robot's distance from far wall (inches)
    //
    // FAR table:   robot is in the far half of the field, Y = 6 to 27
    // CLOSE table: robot is in the close half of the field, Y = 63 to 130
    //
    // GAP Y=27 to Y=63: no real measurements exist yet.
    // Currently handled by a linear blend in the get*WithInterpolation methods below.
    //
    // TO ADD REAL MEASUREMENTS (Option B):
    //   1. Drive robot to several Y positions between 27 and 63 (e.g. Y=35, Y=45, Y=55)
    //   2. At each Y, shoot from 2-3 different X positions and record the correct
    //      velocity and hood value that scores reliably
    //   3. Pick ONE of these two approaches:
    //      a) Add Y rows to FAR_Y/FAR tables (extend FAR_Y to {6, 27, 35, 45, 55})
    //         and add the matching measured values to the farVelocity/Hood matrices
    //      b) Add Y rows to CLOSE_Y/CLOSE tables (extend CLOSE_Y to {45, 55, 63, 88, ...})
    //         and add the matching measured values to the closeVelocity/Hood matrices
    //   4. Once real data covers Y=27 to Y=63, delete the "blend zone" blocks in
    //      getVelocityWithInterpolation() and getHoodWithInterpolation() below —
    //      the if/else if/else becomes a simple two-branch if/else again
    private static final double[] FAR_X = new double[]{43, 71, 100};
    private static final double[] FAR_Y = new double[]{6, 27};
    private static final double[] CLOSE_X = new double[]{38, 61, 85};
    private static final double[] CLOSE_Y = new double[]{63, 88, 111, 130};

    /*
     * FAR TABLE (X = distance/metric on horizontal axis, Y = vertical axis)
     *
     *                X →
     *          43         71         100
     * Y=27    (43,27)   (71,27)   (100,27)
     * Y=6     (43,6)    (71,6)    (100,6)
     *
     * Bilinear interpolation occurs inside the rectangle formed by:
     *   (x1,y1) -------- (x2,y1)
     *      |                |
     *      |     target     |
     *      |      (x,y)     |
     *      |                |
     *   (x1,y2) -------- (x2,y2)
     *
     *
     * CLOSE TABLE
     *
     *                X →
     *          38         61         85
     * Y=135.5 (38,135.5) (61,135.5) (85,135.5)
     * Y=111   (38,111)   (61,111)   (85,111)
     * Y=88    (38,88)    (61,88)    (85,88)
     * Y=63    (38,63)    (61,63)    (85,63)
     *
     * Example:
     * If X=50 and Y=100, interpolation uses the surrounding cell:
     *
     *          38         61
     * Y=111   ●-----------●
     *          |           |
     *          |   X,Y     |
     *          |  (50,100) |
     * Y=88    ●-----------●
     *
     * First interpolate along X at Y=88 and Y=111,
     * then interpolate those two results along Y.
     */

    private static final Interpolation2D farVelocityInterpolation = new BilinearInterpolation(
            FAR_X,
            FAR_Y,
            new double[][]{
                    {1675, 1640},
                    {1675, 1520},
                    {1620, 1520}
            });

    private static final Interpolation2D farHoodInterpolation = new BilinearInterpolation(
            FAR_X,
            FAR_Y,
            new double[][]{
                    {0.47, 0.43},
                    {0.37, 0.33},
                    {0.31, 0.31}
            });

    private static final Interpolation2D closeVelocityInterpolation = new BilinearInterpolation(
            CLOSE_X,
            CLOSE_Y,
            new double[][]{
                    {1520, 1470, 1430, 1400},
                    {1415, 1390, 1380, 1375},
                    {1350, 1330, 1325, 1338},
            }
    );

    private static final Interpolation2D closeHoodInterpolation = new BilinearInterpolation(
            CLOSE_X,
            CLOSE_Y,
            new double[][]{
                    {0.37, 0.38, 0.43, 0.47},
                    {0.36, 0.37, 0.38, 0.43},
                    {0.31, 0.33, 0.36, 0.38},
            }
    );

    public static double getVelocityWithInterpolation(Pose currentPosition, Alliance alliance) {
        Pose pose = alliance == Alliance.RED ? currentPosition : mirror(currentPosition);
        double x = pose.getX(), y = pose.getY();
        if (y <= FAR_Y[FAR_Y.length - 1]) {
            return farVelocityInterpolation.interpolate(clampToTable(x, FAR_X), clampToTable(y, FAR_Y));
        } else if (y >= CLOSE_Y[0]) {
            return closeVelocityInterpolation.interpolate(clampToTable(x, CLOSE_X), clampToTable(y, CLOSE_Y));
        } else {
            // TEMPORARY BLEND ZONE (Y = 27 to 63, no real measurements yet).
            // Linearly interpolates between the FAR table's last row (Y=27)
            // and the CLOSE table's first row (Y=63).
            // t=0 at Y=27 (100% FAR), t=1 at Y=63 (100% CLOSE).
            // Replace this block with real data when available — see instructions at top.
            double t = (y - FAR_Y[FAR_Y.length - 1]) / (CLOSE_Y[0] - FAR_Y[FAR_Y.length - 1]);
            double farVal = farVelocityInterpolation.interpolate(clampToTable(x, FAR_X), FAR_Y[FAR_Y.length - 1]);
            double closeVal = closeVelocityInterpolation.interpolate(clampToTable(x, CLOSE_X), CLOSE_Y[0]);
            return farVal + t * (closeVal - farVal);
        }
    }

    public static double getHoodWithInterpolation(Pose currentPosition, Alliance alliance) {
        Pose pose = alliance == Alliance.RED ? currentPosition : mirror(currentPosition);
        double x = pose.getX(), y = pose.getY();
        if (y <= FAR_Y[FAR_Y.length - 1]) {
            return farHoodInterpolation.interpolate(clampToTable(x, FAR_X), clampToTable(y, FAR_Y));
        } else if (y >= CLOSE_Y[0]) {
            return closeHoodInterpolation.interpolate(clampToTable(x, CLOSE_X), clampToTable(y, CLOSE_Y));
        } else {
            // TEMPORARY BLEND ZONE (Y = 27 to 63, no real measurements yet).
            // Same approach as velocity blend above — replace with real data when available.
            double t = (y - FAR_Y[FAR_Y.length - 1]) / (CLOSE_Y[0] - FAR_Y[FAR_Y.length - 1]);
            double farVal = farHoodInterpolation.interpolate(clampToTable(x, FAR_X), FAR_Y[FAR_Y.length - 1]);
            double closeVal = closeHoodInterpolation.interpolate(clampToTable(x, CLOSE_X), CLOSE_Y[0]);
            return farVal + t * (closeVal - farVal);
        }
    }

    public static double getDistanceToGoal(Pose currentPosition, Alliance alliance) {
        Pose goal = alliance.getGoal();
        return Math.hypot(goal.getX() - currentPosition.getX(), goal.getY() - currentPosition.getY());
    }

    private static double clampToTable(double value, double[] table) {
        return Math.max(table[0], Math.min(table[table.length - 1], value));
    }
}

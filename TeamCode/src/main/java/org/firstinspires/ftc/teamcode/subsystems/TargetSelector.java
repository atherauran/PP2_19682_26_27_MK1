package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.FieldPose.HALF_FIELD_IN;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** 按联盟颜色和机器人在场地中线的哪一侧，选择要瞄准的靶点（Pedro 坐标，单位英寸）。 */
@Configurable
public final class TargetSelector {
    public static double RED_X = 59.0, RED_LEFT_Y = 93.0, RED_RIGHT_Y = 51.0;
    public static double BLUE_X = 85.0, BLUE_LEFT_Y = 51.0, BLUE_RIGHT_Y = 93.0;
    public static double MID = HALF_FIELD_IN;

    private TargetSelector() {}

    public static Pose2D getTarget(double robotYIn, boolean red) {
        double targetX;
        double targetY;
        if (red) {
            targetX = RED_X;
            targetY = robotYIn > MID ? RED_LEFT_Y : RED_RIGHT_Y;
        } else {
            targetX = BLUE_X;
            targetY = robotYIn > MID ? BLUE_RIGHT_Y : BLUE_LEFT_Y;
        }
        return new Pose2D(DistanceUnit.INCH, targetX, targetY, AngleUnit.RADIANS, 0.0);
    }
}

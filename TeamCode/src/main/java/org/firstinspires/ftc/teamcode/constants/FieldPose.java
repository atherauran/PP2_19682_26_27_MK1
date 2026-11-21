package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** 场地尺寸，以及自动结束位置到手动开始位置的交接，统一使用 Pedro 坐标。 */
@Configurable
public final class FieldPose {
    public static final double FIELD_SIZE_IN = 141.5;
    public static final double HALF_FIELD_IN = FIELD_SIZE_IN / 2.0;

    // 手动摆位的默认起点，可在 Panels 修改。位置单位英寸，朝向单位度。
    public static volatile double manualStartX = 71.1;
    public static volatile double manualStartY = 9.0;
    public static volatile double manualStartHeadingDeg = 90.0;

    // 自动结束时保存的位姿。null 表示没有跑过自动，手动用默认起点。
    private static Pose2D autoPose;

    private FieldPose() {}

    public static Pose2D configuredStart() {
        return new Pose2D(DistanceUnit.INCH, manualStartX, manualStartY,
                AngleUnit.DEGREES, manualStartHeadingDeg);
    }

    public static synchronized void saveAutoPose(Pose2D pose) {
        autoPose = pose;
    }

    /** 手动 START 时取走自动留下的位姿，只能取一次，之后单独练习会回到默认起点。 */
    public static synchronized Pose2D takeTeleOpStart() {
        if (autoPose == null) return configuredStart();

        Pose2D start = autoPose;
        autoPose = null;
        return start;
    }
}

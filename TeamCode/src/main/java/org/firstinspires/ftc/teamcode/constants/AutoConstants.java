package org.firstinspires.ftc.teamcode.constants;

import static org.firstinspires.ftc.teamcode.constants.FieldPose.FIELD_SIZE_IN;

import com.pedropathing.geometry.Pose;

/**
 * 自动程序用的位姿和参数。位姿用 Pedro 坐标，朝向单位弧度，时间单位毫秒。
 * 蓝方的数据手动测量，红方由蓝方左右镜像得到。
 */
public final class AutoConstants {
    private AutoConstants() {}

    /** 关于场地中线左右镜像：位置 x 翻转，朝向 θ 变为 180° - θ。 */
    private static Pose mirror(Pose pose) {
        return new Pose(FIELD_SIZE_IN - pose.getX(), pose.getY(),
                Math.toRadians(180) - pose.getHeading());
    }

    /** 曲线控制点只用位置，不带朝向。 */
    private static Pose mirrorPoint(Pose point) {
        return new Pose(FIELD_SIZE_IN - point.getX(), point.getY());
    }

    /** 开始进球到球全部发射出去所需的时间。 */
    public static final long SHOOT_TIME_MS = 450;

    public static final long CLOSE_SHOOT_WAIT_MS = 0;
    public static final double CLOSE_FIRE_DISTANCE = 75.0;
    /** 移动时飞轮保持的转速，按这个距离对应的转速算。 */
    public static final double CLOSE_HOLD_DISTANCE = CLOSE_FIRE_DISTANCE;
    public static final double CLOSE_FIRE_INTAKE_POWER = 0.8;

    public static final long FAR_SHOOT_WAIT_MS = 800;
    public static final double FAR_FIRE_DISTANCE_PRELOAD = 134.4;
    public static final double FAR_HOLD_DISTANCE = 123;
    public static final double FAR_FIRE_INTAKE_POWER = 0.5;

    // ==================== 蓝方 近端 ====================

    public static final double BLUE_CLOSE_FIRE_TURRET = -45;

    private static final double BLUE_INTAKE_START_X = 45;
    private static final double BLUE_INTAKE_END_X = 18.8;
    private static final double BLUE_INTAKE_FIRST_ROW_Y = 85.9;
    private static final double BLUE_INTAKE_SECOND_ROW_Y = 55;
    private static final double BLUE_INTAKE_THIRD_ROW_Y = 36;

    public static final Pose BLUE_CLOSE_START = new Pose(23.52, 121.38, Math.toRadians(141.67));
    /** 近端所有发射点，各轮发射都停在这里。 */
    public static final Pose BLUE_CLOSE_SHOOT = new Pose(61.06, 85.1, Math.toRadians(180));
    public static final Pose BLUE_CLOSE_SHOOT_CONTROL = new Pose(47.95, 60);
    public static final Pose BLUE_CLOSE_SHOOT_FIRST_ROW = new Pose(59.5, 106, Math.toRadians(150));

    public static final Pose BLUE_CLOSE_INTAKE_FIRST_CONTROL = new Pose(50, 78);
    public static final Pose BLUE_CLOSE_INTAKE_FIRST_END = new Pose(BLUE_INTAKE_END_X - 3, BLUE_INTAKE_FIRST_ROW_Y, Math.toRadians(180));
    public static final Pose BLUE_CLOSE_INTAKE_SECOND_CONTROL = new Pose(47.95, 55);
    public static final Pose BLUE_CLOSE_INTAKE_SECOND_END = new Pose(BLUE_INTAKE_END_X - 3, BLUE_INTAKE_SECOND_ROW_Y, Math.toRadians(180));
    public static final Pose BLUE_CLOSE_INTAKE_THIRD_START = new Pose(BLUE_INTAKE_START_X, BLUE_INTAKE_THIRD_ROW_Y, Math.toRadians(180));
    public static final Pose BLUE_CLOSE_INTAKE_THIRD_END = new Pose(BLUE_INTAKE_END_X, BLUE_INTAKE_THIRD_ROW_Y, Math.toRadians(180));

    public static final Pose BLUE_CLOSE_INTAKE_GATE = new Pose(13.27, 59.46, Math.toRadians(155.5));
    public static final Pose BLUE_CLOSE_INTAKE_GATE_CONTROL = new Pose(45.5, 63.5);
    public static final Pose BLUE_CLOSE_INTAKE_GATE_END = new Pose(15.27, 55.46, Math.toRadians(150.5));

    public static final Pose BLUE_CLOSE_PARK = new Pose(55, 65, Math.toRadians(180));

    // ==================== 蓝方 远端 ====================

    public static final double BLUE_FAR_TURRET = -69;

    public static final Pose BLUE_FAR_START = new Pose(59.72, 12.4, Math.toRadians(180.0));
    /** 远端所有发射点，在起点基础上前移 1 英寸。 */
    public static final Pose BLUE_FAR_SHOOT = new Pose(BLUE_FAR_START.getX(), BLUE_FAR_START.getY() + 1, BLUE_FAR_START.getHeading());
    public static final Pose BLUE_FAR_PARK = new Pose(37.8, 16.8, Math.toRadians(180.0));

    public static final Pose BLUE_FAR_INTAKE_HP_1_START = new Pose(13, 13, Math.toRadians(180.0));
    public static final Pose BLUE_FAR_INTAKE_HP_1_MID = new Pose(12, 25, Math.toRadians(131));
    public static final Pose BLUE_FAR_INTAKE_HP_1_END = new Pose(11, 36, Math.toRadians(120));
    public static final Pose BLUE_FAR_INTAKE_THIRD_CONTROL = new Pose(46.5, 39.7);
    public static final Pose BLUE_FAR_INTAKE_THIRD_END = new Pose(14.4, 39, Math.toRadians(180.0));
    public static final Pose BLUE_FAR_INTAKE_TUNNEL_START = new Pose(54.0, 26.5, Math.toRadians(180.0));
    public static final Pose BLUE_FAR_INTAKE_TUNNEL_END = new Pose(12.0, 26.5, Math.toRadians(165.0));

    // ==================== 红方（蓝方镜像） ====================

    public static final double RED_CLOSE_FIRE_TURRET = -BLUE_CLOSE_FIRE_TURRET;

    public static final Pose RED_CLOSE_START = mirror(BLUE_CLOSE_START);
    public static final Pose RED_CLOSE_SHOOT = mirror(BLUE_CLOSE_SHOOT);
    public static final Pose RED_CLOSE_SHOOT_CONTROL = mirrorPoint(BLUE_CLOSE_SHOOT_CONTROL);
    public static final Pose RED_CLOSE_SHOOT_FIRST_ROW = mirror(BLUE_CLOSE_SHOOT_FIRST_ROW);

    public static final Pose RED_CLOSE_INTAKE_FIRST_CONTROL = mirrorPoint(BLUE_CLOSE_INTAKE_FIRST_CONTROL);
    public static final Pose RED_CLOSE_INTAKE_FIRST_END = mirror(BLUE_CLOSE_INTAKE_FIRST_END);
    public static final Pose RED_CLOSE_INTAKE_SECOND_CONTROL = mirrorPoint(BLUE_CLOSE_INTAKE_SECOND_CONTROL);
    public static final Pose RED_CLOSE_INTAKE_SECOND_END = mirror(BLUE_CLOSE_INTAKE_SECOND_END);
    public static final Pose RED_CLOSE_INTAKE_THIRD_START = mirror(BLUE_CLOSE_INTAKE_THIRD_START);
    public static final Pose RED_CLOSE_INTAKE_THIRD_END = mirror(BLUE_CLOSE_INTAKE_THIRD_END);

    public static final Pose RED_CLOSE_INTAKE_GATE = mirror(BLUE_CLOSE_INTAKE_GATE);
    public static final Pose RED_CLOSE_INTAKE_GATE_CONTROL = mirrorPoint(BLUE_CLOSE_INTAKE_GATE_CONTROL);
    public static final Pose RED_CLOSE_INTAKE_GATE_END = mirror(BLUE_CLOSE_INTAKE_GATE_END);

    public static final Pose RED_CLOSE_PARK = mirror(BLUE_CLOSE_PARK);

    public static final double RED_FAR_TURRET = -BLUE_FAR_TURRET;

    public static final Pose RED_FAR_START = mirror(BLUE_FAR_START);
    public static final Pose RED_FAR_SHOOT = mirror(BLUE_FAR_SHOOT);
    public static final Pose RED_FAR_PARK = mirror(BLUE_FAR_PARK);

    public static final Pose RED_FAR_INTAKE_HP_1_START = mirror(BLUE_FAR_INTAKE_HP_1_START);
    public static final Pose RED_FAR_INTAKE_HP_1_MID = mirror(BLUE_FAR_INTAKE_HP_1_MID);
    public static final Pose RED_FAR_INTAKE_HP_1_END = mirror(BLUE_FAR_INTAKE_HP_1_END);
    public static final Pose RED_FAR_INTAKE_THIRD_CONTROL = mirrorPoint(BLUE_FAR_INTAKE_THIRD_CONTROL);
    public static final Pose RED_FAR_INTAKE_THIRD_END = mirror(BLUE_FAR_INTAKE_THIRD_END);
    public static final Pose RED_FAR_INTAKE_TUNNEL_START = mirror(BLUE_FAR_INTAKE_TUNNEL_START);
    public static final Pose RED_FAR_INTAKE_TUNNEL_END = mirror(BLUE_FAR_INTAKE_TUNNEL_END);
}

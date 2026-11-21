package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConstants {
    // 炮台：转过 TURRET_FULL_RANGE_ENCODER 格 = TURRET_FULL_RANGE_DEGREE 度（用 TurretTickCalibration 测）。
    public static double TURRET_FULL_RANGE_DEGREE = 240.0;
    public static double TURRET_FULL_RANGE_ENCODER = 648;
    // 炮台相对车头左右各允许转多少度，超出会拉扯线材。
    public static double TURRET_ABS_RANGE_DEGREE = 120;

    // 距离（英寸）到飞轮转速（ticks/s）的三次拟合：a·x³ + b·x² + c·x + d。
    public static double FLYWHEEL_A = -0.0004036439;
    public static double FLYWHEEL_B = 0.1057019636;
    public static double FLYWHEEL_C = -1.8999025678;
    public static double FLYWHEEL_D = 1153.0825835107;

    // 距离（英寸）到浮板舵机位置的三次拟合，结果限制在 PANEL_MIN ~ PANEL_MAX。
    public static double PANEL_A = 2.39680e-7;
    public static double PANEL_B = -9.05038e-5;
    public static double PANEL_C = 0.015501136306;
    public static double PANEL_D = -0.302168750944;
    public static double PANEL_MIN = 0.0;
    public static double PANEL_MAX = 0.5;

    public static double GATE_OPEN = 0.52;
    public static double GATE_CLOSE = 0.18;
    public static double BLOCK_OPEN = 0.31;
    public static double BLOCK_CLOSE = 0.50;
}

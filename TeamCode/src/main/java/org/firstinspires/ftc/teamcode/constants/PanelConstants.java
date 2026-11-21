package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class PanelConstants {
    // 先空转运行 Flywheel Feedforward Test，把测出的 kS、kV 填在这里。
    // kS 是克服摩擦的电压；kV 是转速每提高 1 tick/s 要多给的电压。
    public static double SHOOTER_KS = 1.36;
    public static double SHOOTER_KV = 0.0053;
    // 再用 VelocityTest 调 kP：比目标慢 100 ticks/s 时补 100 × kP = 1.5 V。
    public static double SHOOTER_KP = 0.019;

    public static double VELOCITY_TOR = 20;
    public static double VELOCITY = 1400;
    public static double POSITION = 0.03;
    public static double FIRE_POWER = 0.75;
}

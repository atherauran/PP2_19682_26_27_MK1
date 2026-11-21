package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.RobotConfigs.TURRET;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.TURRET_ABS_RANGE_DEGREE;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.TURRET_FULL_RANGE_DEGREE;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.TURRET_FULL_RANGE_ENCODER;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

/** 让炮台一直指向场地上的目标，机器人移动或转向时也一样。 */
@Configurable
public class AutoAimSubsystem {

    public static class TurretCommand {
        public boolean hasTarget = false;
        public double targetDist = 0.0;
        public double targetTurretAngle = 0.0;
        public double aimError = 0.0;

        void reset() {
            hasTarget = false;
            targetDist = 0.0;
            targetTurretAngle = 0.0;
            aimError = 0.0;
        }
    }

    public static class EmaFilter {
        private double value;
        private boolean initialized;
        private final double alpha;

        public EmaFilter(double alpha) {
            this.alpha = alpha;
        }

        public double update(double raw) {
            if (!initialized) {
                value = raw;
                initialized = true;
            } else {
                value += alpha * (raw - value);
            }
            return value;
        }

        public double get() { return value; }
        public void reset() { initialized = false; value = 0.0; }
        public boolean isInitialized() { return initialized; }
    }

    @Configurable
    public static class TurretController {

        public static double kP = 10.0, kI = 0.0, kD = 0.5, kF = 0.0001;

        // 前馈：kS 克服静摩擦，kV 维持转速，kA 加减速。用 TurretFullAutoTuner 标定。
        public static double kV = 0.001032, kS = 0.052857, kA = 0.000005;
        // 惯性冲程（转过目标后还会滑多少度）= 线性项 × |角速度| + 二次项 × 角速度²。
        public static double kLinearBraking = 0.039434;
        public static double kQuadraticBraking = 0.0;

        public static double POWER_SIGN = 1.0, MAX_POWER = 1.0;
        public static double TUNING_VOLTAGE = 13.84;

        public static double MIN_SAFE_ANGLE = -TURRET_ABS_RANGE_DEGREE;
        public static double MAX_SAFE_ANGLE = TURRET_ABS_RANGE_DEGREE;
        public static double CENTER_ANGLE = 0.0;
        // 为 true 时 INIT 把炮台当前位置设为 0 度，所以 INIT 前要把炮台摆正对车头。
        public static boolean RESET_ENCODER_ON_INIT = false;

        public static double ANGLE_ALPHA = 0.8, VEL_ALPHA = 0.8, ACCEL_ALPHA = 0.15;

        private DcMotorEx motor;
        private HardwareMap hwMap;

        private final PIDFController pidf = new PIDFController(kP, kI, kD, kF);
        private final EmaFilter angleF = new EmaFilter(ANGLE_ALPHA);
        private final EmaFilter velF = new EmaFilter(VEL_ALPHA);
        private final EmaFilter accelF = new EmaFilter(ACCEL_ALPHA);

        private double lastRawAngle, lastTargetVel;
        private long lastVoltTime;
        private double battery = 12.0;

        public void init(HardwareMap hm) {
            hwMap = hm;
            motor = hm.get(DcMotorEx.class, TURRET);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            if (RESET_ENCODER_ON_INIT) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // 功率由本类自己算，编码器只用来读角度。
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            battery = readBattery();
            pidf.setPIDF(kP, kI, kD, kF);
        }

        /** ffVel 是要提前补上的角速度（度/秒），用来抵消车身转动。 */
        public void update(double curAngle, double tgtAngle, double ffVel, double dt) {
            dt = clampDt(dt);
            pidf.setPIDF(kP, kI, kD, kF);
            refreshBattery();

            tgtAngle = Range.clip(tgtAngle, MIN_SAFE_ANGLE, MAX_SAFE_ANGLE);

            double tgtVel = pidf.calculate(curAngle, tgtAngle) + ffVel;
            double rawAccel = (tgtVel - lastTargetVel) / dt;
            lastTargetVel = tgtVel;
            double filtAccel = accelF.update(rawAccel);

            // 要转得比 0.5 度/秒快才加摩擦补偿，免得停下时来回抖。
            double ks = Math.abs(tgtVel) > 0.5 ? Math.signum(tgtVel) : 0.0;
            double power = filtAccel * kA + tgtVel * kV + ks * kS;
            power *= TUNING_VOLTAGE / battery;
            motor.setPower(Range.clip(POWER_SIGN * power, -MAX_POWER, MAX_POWER));
        }

        /**
         * 炮台还在朝误差的反方向转时，估计刹车时还会滑多少度，把这段加到当前角上再交给 PID，
         * 这样控制器能更早给出反向修正。
         */
        public double predictBraking(double curAngle, double error) {
            double v = velF.get();
            if (Math.signum(error) != Math.signum(v) && Math.abs(v) > 0.5) {
                double d = kLinearBraking * Math.abs(v) + kQuadraticBraking * v * v;
                return curAngle + Math.signum(v) * d;
            }
            return curAngle;
        }

        public void halt() { motor.setPower(0.0); }

        public double rawAngle() {
            return motor.getCurrentPosition() / TURRET_FULL_RANGE_ENCODER * TURRET_FULL_RANGE_DEGREE;
        }
        public double filtAngle() { return angleF.get(); }
        public double filtVel() { return velF.get(); }
        public double getBattery() { return battery; }

        public void stop() {
            motor.setPower(0);
            angleF.reset();
            velF.reset();
            accelF.reset();
            lastTargetVel = 0;
        }

        private void updateFilters(double dt) {
            double raw = rawAngle();
            double velocity = angleF.isInitialized() ? (raw - lastRawAngle) / clampDt(dt) : 0.0;
            angleF.update(raw);
            velF.update(velocity);
            lastRawAngle = raw;
        }

        private void refreshBattery() {
            long now = System.nanoTime();
            if (now - lastVoltTime > 250_000_000L) {
                battery = readBattery();
                lastVoltTime = now;
            }
        }

        private double readBattery() {
            double max = 0;
            for (VoltageSensor s : hwMap.voltageSensor) max = Math.max(max, s.getVoltage());
            return Math.max(8, max > 0 ? max : TUNING_VOLTAGE);
        }

        private static double clampDt(double dt) {
            return Range.clip(dt, 0.005, 0.100);
        }
    }

    private final TurretController turret = new TurretController();
    private final TurretCommand cmd = new TurretCommand();
    private long lastTime = 0;

    public void init(HardwareMap hm) {
        turret.init(hm);
    }

    /**
     * @param robotX     机器人 x 坐标，英寸
     * @param robotY     机器人 y 坐标，英寸
     * @param headingDeg 车头在场地上的朝向，度
     * @param omegaDeg   车头的转速，度/秒
     * @param targetX    靶点 x 坐标，英寸
     * @param targetY    靶点 y 坐标，英寸
     * @param yawOffset  操作手加的角度微调，度
     */
    public TurretCommand update(double robotX, double robotY, double headingDeg, double omegaDeg,
                                double targetX, double targetY, double yawOffset) {
        cmd.reset();

        double dt = sampleTurret();

        double dx = targetX - robotX, dy = targetY - robotY;
        double dist = Math.hypot(dx, dy);
        if (dist < 0.1) {
            // 离靶点太近，方向不可靠，本帧不动。
            turret.halt();
            return cmd;
        }
        cmd.hasTarget = true;
        cmd.targetDist = dist;

        double absTarget = Math.toDegrees(Math.atan2(dy, dx)) + yawOffset;
        double error = normalizeDeg(absTarget - headingDeg - turret.filtAngle());
        double relTarget = turret.filtAngle() + error;

        cmd.targetTurretAngle = relTarget;
        cmd.aimError = error;

        if (relTarget < TurretController.MIN_SAFE_ANGLE || relTarget > TurretController.MAX_SAFE_ANGLE) {
            // 目标超出炮台能转的范围：先回中，下一帧重新判断。回中时仍补偿车身转动。
            cmd.hasTarget = false;
            cmd.targetTurretAngle = TurretController.CENTER_ANGLE;
            turret.update(turret.filtAngle(), TurretController.CENTER_ANGLE, -omegaDeg, dt);
            return cmd;
        }

        double predictedAngle = turret.predictBraking(turret.filtAngle(), error);
        // 车头每秒转 +30°，炮台就要相对车头每秒转 -30°，在场地上才保持不动。
        turret.update(predictedAngle, relTarget, -omegaDeg, dt);
        return cmd;
    }

    public double getCurrentTurretAngle() { return turret.rawAngle(); }
    public double getFilteredTurretAngle() { return turret.filtAngle(); }
    public double getFilteredTurretVelocity() { return turret.filtVel(); }
    public double getCurrentBatteryVoltage() { return turret.getBattery(); }

    /** 不自瞄时让炮台回到相对车头的中心角。炮台随车身一起转，所以不补偿车身转动。 */
    public TurretCommand center() {
        cmd.reset();

        double dt = sampleTurret();
        turret.update(turret.filtAngle(), TurretController.CENTER_ANGLE, 0.0, dt);

        cmd.hasTarget = false;
        cmd.targetTurretAngle = TurretController.CENTER_ANGLE;
        return cmd;
    }

    private double sampleTurret() {
        long now = System.nanoTime();
        double dt = lastTime == 0 ? 0.0001 : (now - lastTime) / 1e9;
        lastTime = now;
        turret.updateFilters(dt);
        return dt;
    }

    public void stop() {
        turret.stop();
        lastTime = 0;
    }

    private static double normalizeDeg(double a) {
        while (a > 180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    }
}

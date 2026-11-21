package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "Flywheel Feedforward Test", group = "Tests")
public class FlywheelFeedforwardTest extends LinearOpMode {
    // 四档由低到高，覆盖这套飞轮常用的转速。每档电压相同的时候，等速度稳定再记录。
    private final double[] volts = {4.0, 5.5, 7.0, 8.5};
    private final double[] speed = new double[volts.length];
    private final Shooter shooter = new Shooter();

    @Override
    public void runOpMode() throws InterruptedException {
        shooter.init(hardwareMap);
        telemetry.addLine("清空飞轮和进球通道。按 START 后自动测四档，全程不送球。");
        telemetry.update();
        waitForStart();
        try {
            for (int i = 0; i < volts.length && opModeIsActive(); i++) {
                speed[i] = measureSpeed(volts[i]);
                if (speed[i] == 0) {
                    telemetry.addLine("标定中止：未取得稳定数据。检查测速线、电池读数和飞轮是否卡住。");
                    break;
                }
            }
        } finally {
            shooter.shooterStop();
        }
        if (!opModeIsActive()) return;
        for (int i = 0; i < volts.length && speed[i] > 0; i++) {
            telemetry.addData("样本 " + (i + 1), "%.1f V / %.0f ticks/s", volts[i], speed[i]);
        }
        if (speed[speed.length - 1] > 0) showFit();
        telemetry.update();
        while (opModeIsActive()) sleep(100); // 已停转，留出抄写结果的时间。
    }

    private double measureSpeed(double testVolts) throws InterruptedException {
        ElapsedTime stage = new ElapsedTime();
        ElapsedTime window = new ElapsedTime();
        int count = 0;
        double sum = 0, low = Double.POSITIVE_INFINITY, high = 0;
        while (opModeIsActive() && stage.seconds() < 8) {
            shooter.setFlywheelTestVoltage(testVolts);
            double current = shooter.flywheelSpeed;
            telemetry.addData("标定电压 / 估算输出 (V)", "%.1f / %.1f", testVolts, shooter.appliedFlywheelVolts);
            telemetry.addData("当前转速 (ticks/s)", current);
            telemetry.update();
            if (shooter.hasFlywheelFeedbackFault() || shooter.batteryVolts == 0) return 0;

            // 先留 1.5 秒加速。输出被限幅或读数消失时，本段样本作废，重新等一段。
            if (stage.seconds() < 1.5 || current < 20 || Math.abs(shooter.appliedFlywheelVolts - testVolts) > 0.05) {
                count = 0;
            } else {
                if (count == 0) {
                    window.reset();
                    sum = 0;
                    low = high = current;
                }
                sum += current;
                count++;
                low = Math.min(low, current);
                high = Math.max(high, current);
                if (window.seconds() >= 0.8 && count >= 10) {
                    double mean = sum / count;
                    if (high - low <= Math.max(30, 0.025 * mean)) return mean;
                    count = 0;
                }
            }
            sleep(20);
        }
        return 0;
    }

    private void showFit() {
        // 在纸上，横轴画转速，纵轴画电压。找一条最贴近四个点的直线：
        // 每增加 1 tick/s 要多给多少伏，就是斜率 kV；直线在纵轴的起点就是 kS。
        int n = volts.length;
        double sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;
        for (int i = 0; i < n; i++) {
            if (i > 0 && speed[i] <= speed[i - 1]) {
                telemetry.addLine("电压升高后转速没有升高，请检查两侧读数并重测。");
                return;
            }
            sumX += speed[i];
            sumY += volts[i];
            sumXX += speed[i] * speed[i];
            sumXY += speed[i] * volts[i];
        }
        double kV = (n * sumXY - sumX * sumY) / (n * sumXX - sumX * sumX);
        double kS = (sumY - kV * sumX) / n;
        if (kS < 0 && kS >= -0.2) { // 测量误差可能让很小的摩擦电压略低于零。
            kS = 0;
            kV = sumXY / sumXX;
        }
        for (int i = 0; i < n; i++) {
            if (!Double.isFinite(kV) || kV <= 0 || !(kS >= 0)
                    || Math.abs(kS + kV * speed[i] - volts[i]) > 0.4) {
                telemetry.addLine("四个点与模型不符，请检查飞轮和编码器后重测。");
                return;
            }
        }
        telemetry.addData("SHOOTER_KS", "%.4f", kS);
        telemetry.addData("SHOOTER_KV", "%.7f", kV);
    }
}

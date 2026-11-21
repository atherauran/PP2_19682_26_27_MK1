package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.RobotConfigs.LEFT_SHOOTER;
import static org.firstinspires.ftc.teamcode.constants.RobotConfigs.PANEL;
import static org.firstinspires.ftc.teamcode.constants.RobotConfigs.RIGHT_SHOOTER;
import static org.firstinspires.ftc.teamcode.constants.RobotConfigs.TURRET;
import static org.firstinspires.ftc.teamcode.constants.PanelConstants.SHOOTER_KP;
import static org.firstinspires.ftc.teamcode.constants.PanelConstants.SHOOTER_KS;
import static org.firstinspires.ftc.teamcode.constants.PanelConstants.SHOOTER_KV;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Shooter {
    public DcMotorEx leftShooter;
    public DcMotorEx rightShooter;
    public DcMotorEx turret;
    public Servo panel;

    public static double targetVelocity = 0;
    public static double targetPanel = 0;

    // 刚启动时编码器读数是 0。先用小功率起转，最多等 0.5 秒，还读不到转速就判定测速故障。
    private static final double START_POWER_LIMIT = 0.20;
    private static final double FEEDBACK_TIMEOUT_SECONDS = 0.50;
    // 低于这个转速（ticks/s）的读数视为没有读数。
    private static final double MIN_VALID_SPEED = 20.0;
    private static final double HOLD_VELOCITY = 800;

    private final ElapsedTime feedbackTimer = new ElapsedTime();
    private int flywheelDirection; // 1 正转，-1 反转，0 关闭
    private boolean sawSpeed; // 本次开启后是否已读到有效转速
    private boolean feedbackFault; // 测速故障，关闭再开启飞轮才能清除
    private Iterable<VoltageSensor> batterySensors;

    public double batteryVolts, appliedFlywheelVolts, flywheelSpeed;

    public void init(HardwareMap hardwareMap) {
        batterySensors = hardwareMap.voltageSensor;
        leftShooter = hardwareMap.get(DcMotorEx.class, LEFT_SHOOTER);
        rightShooter = hardwareMap.get(DcMotorEx.class, RIGHT_SHOOTER);
        turret = hardwareMap.get(DcMotorEx.class, TURRET);
        panel = hardwareMap.get(Servo.class, PANEL);

        // 功率由本类自己算，Hub 不再单独调速。这个模式仍然能读转速。
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setPositionPIDFCoefficients(15);

        shooterStop();
    }

    public void reset() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setPower(0);
    }

    /** 设定飞轮目标转速（ticks/s），负数反转，0 或无效值停转。要每帧调用。 */
    public void setShooterVelocity(double velocity) {
        if (velocity == 0 || !Double.isFinite(velocity)) {
            shooterStop();
            return;
        }
        updateFlywheels(velocity, 0);
    }

    /** 标定用：只给固定电压，让飞轮自己升到稳定转速。测速故障保护和比赛时相同。 */
    public void setFlywheelTestVoltage(double volts) {
        if (volts > 0 && Double.isFinite(volts)) updateFlywheels(0, volts);
        else shooterStop();
    }

    private void updateFlywheels(double velocity, double testVolts) {
        int direction = velocity < 0 ? -1 : 1;
        if (flywheelDirection != direction) {
            flywheelDirection = direction;
            sawSpeed = false;
            feedbackTimer.reset();
        }

        double target = Math.abs(velocity);
        flywheelSpeed = getShooterVelocity();
        double actual = flywheelSpeed * direction;
        batteryVolts = readBatteryVolts();

        boolean calibrating = testVolts > 0;
        double baseVolts = calibrating ? testVolts : SHOOTER_KS + SHOOTER_KV * target;
        double power = 0;
        if (!feedbackFault && batteryVolts > 0) {
            double minSpeed = calibrating ? MIN_VALID_SPEED : Math.min(MIN_VALID_SPEED, target * 0.1);
            if (actual > 0 && actual >= minSpeed) {
                sawSpeed = true;
                feedbackTimer.reset();
                double volts = baseVolts;
                if (!calibrating) volts += SHOOTER_KP * (target - actual);
                power = volts / batteryVolts;
            } else if (feedbackTimer.seconds() >= FEEDBACK_TIMEOUT_SECONDS) {
                // 半秒读不到转速：停机，直到操作者关闭再开启飞轮。
                feedbackFault = true;
            } else if (!sawSpeed) {
                power = Math.min(baseVolts / batteryVolts, START_POWER_LIMIT);
            }
        }

        // 左右电机收到同一个功率，右电机的反向已在 init 里设置。
        power = Double.isFinite(power) ? direction * Range.clip(power, 0, 1) : 0;
        appliedFlywheelVolts = power * batteryVolts;
        leftShooter.setPower(power);
        rightShooter.setPower(power);
    }

    private double readBatteryVolts() {
        for (VoltageSensor sensor : batterySensors) {
            double volts = sensor.getVoltage();
            if (volts >= 6 && volts <= 16) return volts;
        }
        return 0;
    }

    /**
     * 飞轮转速（ticks/s）。左右各读一次，一边读不到也能工作。
     * 两边都有读数时取沿目标方向较快的一边，避免接触不良的低读数把功率拉高。
     */
    public double getShooterVelocity() {
        double left = leftShooter.getVelocity();
        double right = rightShooter.getVelocity();
        if (!Double.isFinite(left)) left = 0;
        if (!Double.isFinite(right)) right = 0;
        if (flywheelDirection > 0) return Math.max(0, Math.max(left, right));
        if (flywheelDirection < 0) return Math.min(0, Math.min(left, right));
        return Math.abs(left) >= Math.abs(right) ? left : right;
    }

    public boolean hasFlywheelFeedbackFault() {
        return feedbackFault;
    }

    public void shooterHold() {
        setShooterVelocity(HOLD_VELOCITY);
    }

    public void shooterStop() {
        flywheelDirection = 0;
        sawSpeed = false;
        feedbackFault = false;
        appliedFlywheelVolts = 0;
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }

    public void setShooter(double panel, double velocity) {
        panelTo(Range.clip(panel, PANEL_MIN, PANEL_MAX));
        setShooterVelocity(velocity);
    }

    public void setShooterByDis(double distance) {
        setShooterByDis(distance, 0, 0);
    }

    public void setShooterByDis(double distance, double velocityCorrection, double panelCorrection) {
        updateTargets(distance, velocityCorrection, panelCorrection);
        setShooter(targetPanel, targetVelocity);
    }

    /** 只更新目标值，不动飞轮和浮板，用于飞轮关闭时也要显示目标。 */
    public void updateTargets(double distance, double velocityCorrection, double panelCorrection) {
        targetVelocity = cubic(FLYWHEEL_A, FLYWHEEL_B, FLYWHEEL_C, FLYWHEEL_D, distance) + velocityCorrection;
        targetPanel = cubic(PANEL_A, PANEL_B, PANEL_C, PANEL_D, distance) + panelCorrection;
    }

    public double calculateIntakePower() {
        if (targetVelocity < 1540.0) return 1;
        if (targetVelocity < 1800.0) return 0.85;
        return 0.75;
    }

    private static double cubic(double a, double b, double c, double d, double x) {
        return a * Math.pow(x, 3) + b * Math.pow(x, 2) + c * x + d;
    }

    public void panelTo(double pos) {
        panel.setPosition(pos);
    }

    public void turretRotateLeft() {
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setPower(0.5);
    }

    public void turretRotateRight() {
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setPower(-0.5);
    }

    public void turretRotateStop() {
        turret.setPower(0);
    }

    public int getTurretPosition() {
        return turret.getCurrentPosition();
    }

    public double getTurretDegree() {
        return getTurretPosition() / TURRET_FULL_RANGE_ENCODER * TURRET_FULL_RANGE_DEGREE;
    }

    public void turretToDegree(double degree) {
        int position = (int) (degree * TURRET_FULL_RANGE_ENCODER / TURRET_FULL_RANGE_DEGREE);
        turret.setTargetPosition(position);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
    }
}

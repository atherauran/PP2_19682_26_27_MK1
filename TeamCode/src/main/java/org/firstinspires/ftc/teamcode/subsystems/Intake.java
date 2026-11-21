package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.RobotConfigs.BLOCK_GATE;
import static org.firstinspires.ftc.teamcode.constants.RobotConfigs.GATE;
import static org.firstinspires.ftc.teamcode.constants.RobotConfigs.INTAKE;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.BLOCK_CLOSE;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.BLOCK_OPEN;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.GATE_CLOSE;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.GATE_OPEN;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotorEx intake;
    private Servo gate, blockGate;

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, INTAKE);
        gate = hardwareMap.get(Servo.class, GATE);
        blockGate = hardwareMap.get(Servo.class, BLOCK_GATE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void intakeIn() {
        intake.setPower(1);
    }

    public void intakeIn(double power) {
        intake.setPower(power);
    }

    public void intakeFire(double power) {
        intake.setPower(power);
    }

    public void intakeOut() {
        intake.setPower(-1);
    }

    public void intakeOut(double power) {
        intake.setPower(-power);
    }

    public void intakeStop() {
        intake.setPower(0);
    }

    public void gateOpen() {
        gate.setPosition(GATE_OPEN);
        blockGate.setPosition(BLOCK_OPEN);
    }

    public void gateClose() {
        gate.setPosition(GATE_CLOSE);
        blockGate.setPosition(BLOCK_CLOSE);
    }
}

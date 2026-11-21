package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Drivetrain drivetrain = new Drivetrain();
    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();

    public void init(HardwareMap hardwareMap) {
        drivetrain.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
    }

    public void autoInit(HardwareMap hardwareMap) {
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        shooter.reset();
    }
}

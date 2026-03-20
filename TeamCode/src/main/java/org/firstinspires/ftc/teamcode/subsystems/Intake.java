package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Intake {
    MotorEx intakeMotor;

    public Intake (HardwareMap hardwareMap) {
        intakeMotor = new MotorEx(hardwareMap, "intakeMotor");
    }

    public void intake () {
        intakeMotor.set(1);
    }

    public void idle () {
        intakeMotor.set(0);
    }
}

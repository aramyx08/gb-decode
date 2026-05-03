package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {
    DcMotor intakeMotor;

    public Intake (HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
    }

    public void intake () {
        intakeMotor.setPower(1);
    }

    public void idle () {
        intakeMotor.setPower(0);
    }
}

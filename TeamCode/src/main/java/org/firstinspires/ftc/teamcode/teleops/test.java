package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="test", group="Iterative Opmode")
public class test extends OpMode {
    Servo kick0;
    Servo kick1;
    Servo kick2;

    public void init() {
        kick0 = hardwareMap.get(Servo.class, "lifter0");
        kick1 = hardwareMap.get(Servo.class, "lifter1");
        kick2 = hardwareMap.get(Servo.class, "lifter2");
    }

    public void loop () {
        // good
        if(gamepad1.a) {
            kick2.setPosition(0.8);
        } else {
            kick2.setPosition(0);
        }

        if(gamepad1.b) {
            kick1.setPosition(0.3);
        } else {
            kick1.setPosition(1);
        }

        if(gamepad1.y) {
            kick0.setPosition(0.7);
        } else {
            kick0.setPosition(0);
        }
    }
}

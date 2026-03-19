package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.teamcode.utilities.TelemetryDebug;

public class Shooter {
    private Motor topFlywheelMotor;
    private Motor bottomFlywheelMotor;
    private MotorGroup flywheel;
    private Limelight3A limelight;
    private TelemetryDebug debug;
    private PIDFController velocityController = new PIDFController(0.002, 0.00005, 0.000003, 0.00022);

    public Shooter (HardwareMap hardwareMap, boolean isRed, TelemetryDebug debug) {
        topFlywheelMotor = new Motor(hardwareMap, "topMotor", 28, 6000);
        bottomFlywheelMotor = new Motor(hardwareMap, "bottomMotor", 28, 6000);
        flywheel = new MotorGroup(topFlywheelMotor, bottomFlywheelMotor);

        flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        flywheel.setRunMode(Motor.RunMode.RawPower);
    }


}

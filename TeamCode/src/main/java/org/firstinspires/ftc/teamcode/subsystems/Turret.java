package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;

import org.firstinspires.ftc.teamcode.utilities.TelemetryDebug;

public class Turret {
    private CRServoEx leftTurretServo;
    private CRServoEx rightTurretServo;
    private CRServoGroup turret;
    private Limelight3A limelight;
    private TelemetryDebug debug;

    final PIDFController limelightPIDF = new PIDFController(0.012, 0.014, 0.00015, 0.005);

    public Turret (HardwareMap hardwareMap, boolean isRed, TelemetryDebug debug) {
        this.debug = debug;

        // Declares and sets up turret servos
        leftTurretServo = new CRServoEx(hardwareMap, "leftTurretServo");
        rightTurretServo = new CRServoEx(hardwareMap, "rightTurretServo");

        turret = new CRServoGroup(leftTurretServo, rightTurretServo);

        // Declares and sets up limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(isRed ? 0 : 1);
        limelight.start();
    }

    public void aim() {
        limelight.start();
        // Retrieve limelight data
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            double power = limelightPIDF.calculate(llResult.getTx(), 0);
            power = Math.max(-0.7, Math.min(0.7, power));
            turret.set(power);

            debug.createWatcher("Tx", llResult.getTx());
            debug.createWatcher("Turret Power", power);
        } else {
            turret.set(0);
        }
    }

    public void idle() {
        turret.set(0);
        limelight.pause();
    }

}

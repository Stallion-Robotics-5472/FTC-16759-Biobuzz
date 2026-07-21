package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class Testing extends LinearOpMode {
    ServoImplEx axon;

    @Override
    public void runOpMode() throws InterruptedException {
        axon = hardwareMap.get(ServoImplEx.class, "axon");

        axon.setPwmRange(new PwmControl.PwmRange(500, 2500));

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.a){axon.setPosition(0);}
            else {axon.setPosition(0.5);}
        }
    }
}

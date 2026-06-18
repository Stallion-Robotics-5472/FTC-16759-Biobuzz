package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends Constants{
    final DcMotor intake;
    final Telemetry telemetry;
    ElapsedTime outtakeTimer = new ElapsedTime();
    Gamepad opCon;
    public boolean initDone = false;
    public IntakeSubsystem(Gamepad opCon, HardwareMap hardwareMap, Telemetry telemetry){
        intake = hardwareMap.get(DcMotor.class,"intake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        opCon.setTriggerThreshold(triggerThresh);

        this.opCon = opCon;

        this.telemetry = telemetry;

        initDone = true;
    } // initialization

    public void collect(){
        boolean intakeOn = opCon.right_trigger_pressed;

        if (intakeOn) {
            intake.setPower(1);
            if (intake.getPower() < 0.8){
                outtakeTimer.reset();
                while (outtakeTimer.milliseconds() < 1500) {
                    intake.setPower(-1);
                }
            }
        } else {
            intake.setPower(0);
        }
    }

    public boolean intakeIsOn(){
        return intake.getPower() < 0.2 && intake.getPower() > -0.2;
    }
}
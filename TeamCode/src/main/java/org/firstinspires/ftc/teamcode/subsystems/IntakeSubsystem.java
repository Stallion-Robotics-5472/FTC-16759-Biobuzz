package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem {
    final DcMotorEx intake;
    final Telemetry telemetry;
    Constants constants = new Constants();
    Gamepad opCon;
    public boolean intakeOn = false;
    public IntakeSubsystem(Gamepad opCon, HardwareMap hardwareMap, Telemetry telemetry){
        intake = hardwareMap.get(DcMotorEx.class,"intake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        opCon.setTriggerThreshold(constants.triggerThresh);

        this.opCon = opCon;

        this.telemetry = telemetry;
    } // initialization

    public void collect(){
        intakeOn = opCon.right_trigger_pressed;

        if (intakeOn){
            intake.setVelocity(2700);
        } else {
            intake.setVelocity(0);
        }
    }
}
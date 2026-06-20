package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSubsystem extends Constants{
    final DcMotorEx intake;
    final Telemetry telemetry;
    ElapsedTime outtakeTimer = new ElapsedTime();
    Gamepad opCon;
    public IntakeSubsystem(Gamepad opCon, HardwareMap hardwareMap, Telemetry telemetry){
        intake = hardwareMap.get(DcMotorEx.class,"intake");

        intake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setVelocityPIDFCoefficients(intakekP, intakekI, intakekD, 0);

        opCon.setTriggerThreshold(triggerThresh);

        this.opCon = opCon;

        this.telemetry = telemetry;
    } // initialization

    public void collect(){
        boolean intakeOn = opCon.right_trigger_pressed;

        if (intakeOn && intake.getCurrent(CurrentUnit.AMPS) > 9) {
            outtakeTimer.reset();
            if (outtakeTimer.milliseconds() < 250) {
                intake.setVelocity(-2000);
            }
        } else if (intakeOn) {
            intake.setVelocity(2200);
        } else {
            intake.setVelocity(0);
        }
    }

    public void disableIntake(){
        intake.setMotorDisable();
    }

    public void enableIntake(){
        intake.setMotorEnable();
    }
}
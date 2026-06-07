package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IntakeSubsystem {
    boolean intakeOn;
    final DcMotor intake;
    final Telemetry telemetry;
    Constants constants = new Constants();
    Gamepad opCon;
    public IntakeSubsystem(Gamepad opCon, HardwareMap hardwareMap, Telemetry telemetry){
        intake = hardwareMap.dcMotor.get("intake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.opCon = opCon;

        this.telemetry = telemetry;
    } // initialization

    public void collect(){
        if (intakeOn){
            intake.setPower(1);
        }else{
            intake.setPower(0);
        }

        if (opCon.aWasPressed()){
            intakeOn = !intakeOn;
        }
    }

    public void teleData(){
        telemetry.addLine("--INTAKE TELE--");
        telemetry.addData("IntakeOn", intakeOn);
        telemetry.addLine();
    }
}
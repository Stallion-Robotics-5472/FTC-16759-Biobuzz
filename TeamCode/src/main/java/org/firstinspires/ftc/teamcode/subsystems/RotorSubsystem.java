package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Constants;

public class RotorSubsystem extends Constants {
    DcMotor rotor;
    DcMotor rollers;
    enum RotorStates{
        IDLE,
        FEED
    }
    RotorStates rotorState;

    public RotorSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        rotor = hardwareMap.get(DcMotor.class, "rotor");
        rollers = hardwareMap.get(DcMotor.class, "rollers");

        rotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollers.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void swapStates(boolean isShooting){
        switch (rotorState){
            case IDLE:
                rotor.setPower(-0.5);
                rollers.setPower(-0.5);
                if (isShooting){
                    rotorState = RotorStates.FEED;
                }
                break;
            case FEED:
                rotor.setPower(1);
                rollers.setPower(1);
                if (!isShooting){
                    rotorState = RotorStates.IDLE;
                }
                break;
        }
    }
}

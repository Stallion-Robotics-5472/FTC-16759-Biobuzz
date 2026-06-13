package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem {
    Limelight3A ll;
    LLResult result;
    DcMotorEx pivot;
    Gamepad opCon;
    final Telemetry telemetry;
    Constants constants = new Constants();
    double commandRot = 0;
    public TurretSubsystem(Gamepad opCon, HardwareMap hardwareMap, Telemetry telemetry){
        ll = hardwareMap.get(Limelight3A.class,"limelight");
        pivot = hardwareMap.get(DcMotorEx.class,"pivot");

        ll.start();

        result = ll.getLatestResult();

        this.opCon = opCon;

        this.telemetry = telemetry;
    } // initialization

    public void turn(){
        if (result.isValid() && ll != null){
            if (result.getTx() != 0){
                    pivot.setVelocity(result.getTx() * 500);
                }
            }
        }
    }
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DumperSubsystem {
    final DcMotorEx leftElev;
    final DcMotorEx rightElev;
    final Telemetry telemetry;
    final Servo leftPivot;
    final Servo rightPivot;
    Constants constants = new Constants();
    Gamepad opCon;
    public int commandPos = 0;
    int veloMult = 1;
    public DumperSubsystem(Gamepad opCon, HardwareMap hardwareMap, Telemetry telemetry){
        leftElev = hardwareMap.get(DcMotorEx.class,"leftElev");
        rightElev = hardwareMap.get(DcMotorEx.class,"rightElev");
        leftPivot = hardwareMap.get(Servo.class,"leftPivot");
        rightPivot = hardwareMap.get(Servo.class,"rightPivot");

//        leftElev.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightElev.setDirection(DcMotorSimple.Direction.REVERSE);
        leftPivot.setDirection(Servo.Direction.FORWARD);
        rightPivot.setDirection(Servo.Direction.REVERSE);

        leftElev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftElev.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightElev.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftPivot.scaleRange(constants.levelAng,constants.tipAng);
        rightPivot.scaleRange(constants.levelAng,constants.tipAng);

        this.opCon = opCon;

        this.telemetry = telemetry;
    } // initialization

    public void tuck(){
        goToPos(0);
        tip(0);
    }

    public void raise(){
        if (opCon.right_bumper){
            commandPos = constants.maxExt;
        } else if (opCon.left_bumper){
            commandPos = constants.halfExt;
        }
        goToPos(commandPos);
        tip(0);
    }

    public void dump(){
        goToPos(commandPos);
        tip(1);
    }

    void goToPos(int pos){
        if (pos > constants.maxExt){
            pos = constants.maxExt;
        } else if (pos < constants.tuckedExt) {
            pos = constants.tuckedExt;
        }

        if (leftElev.getCurrentPosition() != pos || rightElev.getCurrentPosition() != pos) {
                int error = Math.abs(pos - leftElev.getCurrentPosition());

                if (pos > leftElev.getCurrentPosition()){veloMult = 1;}
                else if (pos < leftElev.getCurrentPosition()){veloMult = -1;}

                leftElev.setVelocity(error * constants.elevKP * veloMult);
                rightElev.setVelocity(error * constants.elevKP * veloMult);
        }
    }

    void tip(double angle){
        leftPivot.setPosition(angle);
        rightPivot.setPosition(angle);
    }
}
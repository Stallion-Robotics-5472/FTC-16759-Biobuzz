package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DumperSubsystem extends Constants{
    final DcMotorEx leftElev;
    final DcMotorEx rightElev;
    final Servo hopper;
    final Servo intakeFlap;
    final Telemetry telemetry;
    Gamepad opCon;
    public int commandPos = 0;
    ElevatorTrapezoidalMotionProfile elevator;
    public DumperSubsystem(Gamepad opCon, HardwareMap hardwareMap, Telemetry telemetry){
        leftElev = hardwareMap.get(DcMotorEx.class,"leftElev");
        rightElev = hardwareMap.get(DcMotorEx.class,"rightElev");
        hopper = hardwareMap.get(Servo.class, "hopper");
        intakeFlap = hardwareMap.get(Servo.class, "intakeFlap");

//        leftElev.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightElev.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator = new ElevatorTrapezoidalMotionProfile(
                leftElev, rightElev,
                elevkP, elevkI, elevkD, elevkF,
                maxVel, maxAccel
        );

        elevator.resetEncoders();

        hopper.scaleRange(0,1);
        intakeFlap.scaleRange(0,1);

        // 0 = closed, 1 = open

        this.opCon = opCon;

        this.telemetry = telemetry;
    } // initialization

    public void tuck(){
        if (elevator.getCurrentPositionMM() != tuckedExt || intakeFlap.getPosition() != 1 || hopper.getPosition() != 0) {
            elevatorSetHeight(tuckedExt);
            intakeFlap.setPosition(1);
            hopper.setPosition(0);
        }
    }

    public void raise(){
        if (opCon.dpadUpWasPressed() && commandPos != highExt) {commandPos = highExt;}
        else if (opCon.dpadRightWasPressed() && commandPos != midExt) {commandPos = midExt;}
        else if (opCon.dpadDownWasPressed() && commandPos != lowExt) {commandPos = lowExt;}

        if (elevator.getCurrentPositionMM() != commandPos || intakeFlap.getPosition() != 0) {
            elevatorSetHeight(commandPos);
            intakeFlap.setPosition(0);
        }
    }

    public void dump(){
        if (hopper.getPosition() != 1){
            hopper.setPosition(1);
        }
    }

    void goToPos(int pos){
        if (pos > highExt){
            pos = highExt;
        } else if (pos < tuckedExt) {
            pos = tuckedExt;
        }

        if (leftElev.getCurrentPosition() != pos || rightElev.getCurrentPosition() != pos) {
                int error = Math.abs(pos - leftElev.getCurrentPosition());

                int veloMult;

                if (pos > leftElev.getCurrentPosition()){veloMult = 1;}
                else {veloMult = -1;}

                leftElev.setVelocity(error * elevkP * veloMult);
                rightElev.setVelocity(error * elevkP * veloMult);
        }
    }

    void elevatorSetHeight(double targetMM) {
        elevator.goToPos(targetMM);

        while (elevator.isBusy()) {
            elevator.update();
        }
    }
}
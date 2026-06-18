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
    final Telemetry telemetry;
    Gamepad opCon;
    public int commandPos = 0;
    int veloMult = 1;
    ElevatorTrapezoidalMotionProfile elevator;
    public boolean initDone = false;
    public DumperSubsystem(Gamepad opCon, HardwareMap hardwareMap, Telemetry telemetry){
        leftElev = hardwareMap.get(DcMotorEx.class,"leftElev");
        rightElev = hardwareMap.get(DcMotorEx.class,"rightElev");
        hopper = hardwareMap.get(Servo.class, "hopper");

//        leftElev.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightElev.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator = new ElevatorTrapezoidalMotionProfile(
                leftElev, rightElev,
                elevkP, elevkI, elevkD, elevkF,
                maxVel, maxAccel
        );

        elevator.resetEncoders();

        this.opCon = opCon;

        this.telemetry = telemetry;

        elevator.resetEncoders();

        this.initDone = true;
    } // initialization

    public void tuck(){
        elevatorSetHeight(tuckedExt);
    }

    public void raise(){
        if (opCon.dpadUpWasPressed()) {commandPos = highExt;
        } else if (opCon.dpadRightWasPressed()) {commandPos = midExt;
        } else if (opCon.dpadDownWasPressed()) {commandPos = lowExt;}

        elevatorSetHeight(commandPos);
    }

    void goToPos(int pos){
        if (pos > highExt){
            pos = highExt;
        } else if (pos < tuckedExt) {
            pos = tuckedExt;
        }

        if (leftElev.getCurrentPosition() != pos || rightElev.getCurrentPosition() != pos) {
                int error = Math.abs(pos - leftElev.getCurrentPosition());

                if (pos > leftElev.getCurrentPosition()){veloMult = 1;}
                else if (pos < leftElev.getCurrentPosition()){veloMult = -1;}

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
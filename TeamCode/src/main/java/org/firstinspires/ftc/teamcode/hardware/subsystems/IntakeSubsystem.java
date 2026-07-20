package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Constants;

public class IntakeSubsystem extends Constants {
    final DcMotorEx intake;
    final Telemetry telemetry;
    ElapsedTime outtakeTimer = new ElapsedTime();
    enum IntakeStates{
        IDLE,
        COLLECT,
        SPIT
    }
    IntakeStates intakeState;
    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        intake = hardwareMap.get(DcMotorEx.class,"intake");

        intake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeState = IntakeStates.IDLE;

        this.telemetry = telemetry;
    } // initialization

    public void runIntake(boolean intakeOn){
        switch (intakeState){
            case IDLE:
                if (intake.getVelocity() != 0) { intake.setVelocity(0); }

                if (intakeOn){
                    intakeState = IntakeStates.COLLECT;
                }
            case COLLECT:
                if (intake.getVelocity() != 2200) { intake.setVelocity(2200); }

                if (!intakeOn){
                    intakeState = IntakeStates.IDLE;
                } else if (intake.getCurrent(CurrentUnit.MILLIAMPS) > 1500){
                    intakeState = IntakeStates.SPIT;
                }
            case SPIT:
                outtakeTimer.reset();
                while (outtakeTimer.milliseconds() < 250) {
                    intake.setVelocity(-2000);
                }
                intakeState = IntakeStates.COLLECT;
        }
    }

    public void disableIntake(){
        if (intake.isMotorEnabled()) {
            intake.setMotorDisable();
            intakeState = IntakeStates.IDLE;
        }
    }

    public void enableIntake(){
        if (!intake.isMotorEnabled()) {
            intake.setMotorEnable();
        }
    }
}
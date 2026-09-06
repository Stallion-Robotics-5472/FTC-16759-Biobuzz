package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Constants;
import org.firstinspires.ftc.teamcode.hardware.InterpolatingDoubleTreeMap;

public class TurretSubsystem extends Constants {
    DcMotorEx shooter1;
    DcMotorEx shooter2;
    ServoImplEx pivot1;
    ServoImplEx pivot2;
    ServoImplEx hood;
    enum ShooterStates{
        IDLE,
        SPIN_UP,
        SHOOT
    }
    ShooterStates shooterState;
    InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap pivotMap = new InterpolatingDoubleTreeMap();
    double distX = 0;
    double distY = 0;
    double curX = 0;
    double curY = 0;
    double curTheta = 0;
    double distanceToGoal = 0;
    double reqVelocity = 0;

    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        pivot1 = hardwareMap.get(ServoImplEx.class, "pivot1");
        pivot2 = hardwareMap.get(ServoImplEx.class, "pivot2");
        hood = hardwareMap.get(ServoImplEx.class, "hood");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setVelocityPIDFCoefficients(shooterkP, shooterkI, shooterkD, 0);
        shooter2.setVelocityPIDFCoefficients(shooterkP, shooterkI, shooterkD, 0);

        pivot1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        pivot2.setPwmRange(new PwmControl.PwmRange(500, 2500));

        velocityMap.put(12.0, 1000);
        velocityMap.put(36.0, 1500);
        velocityMap.put(50.0, 2200);

        hoodMap.put(12.0, 0.3);
        hoodMap.put(36.0, 0.5);
        hoodMap.put(50.0, 0.6);
    }

    void setShooter(double velocity){
        this.reqVelocity = velocity;
        if (!isSpunUp()) {
            shooter1.setVelocity(reqVelocity);
            shooter2.setVelocity(reqVelocity);
        }
    }

    public boolean isSpunUp(){
        return shooter1.getVelocity() == this.reqVelocity;
    }

    void setPivot(double ang){
        pivot1.setPosition(ang);
        pivot2.setPosition(ang);
    }

    void setHood(double ang){
        hood.setPosition(ang);
    }

    public void updatePos(double curX, double curY, double curTheta){
        this.curX = curX;
        this.curY = curY;
        this.curTheta = curTheta;
    }

    double getMeasuresToGoal(boolean distanceReq){
        distX = Math.abs(goalX - curX);
        distY = Math.abs(goalY - curY);
        if (distanceReq) {
            distanceToGoal = Math.sqrt(distX*distX + distY*distY);
            return distanceToGoal;
        } else {
            return Math.atan(distY/distX);
        }
    }

    public void shoot(boolean shotRequested, double curX, double curY){
        switch(shooterState){
            case IDLE:
                setShooter(0);
                setHood(0);
                if (shotRequested){
                    shooterState = ShooterStates.SPIN_UP;
                }
                break;
            case SPIN_UP:
                setShooter(velocityMap.get(getMeasuresToGoal(true)));
                setHood(hoodMap.get(getMeasuresToGoal(true)));
                if (isSpunUp() && shotRequested){
                    shooterState = ShooterStates.SHOOT;
                } else if (!shotRequested){
                    shooterState = ShooterStates.IDLE;
                }
                break;
            case SHOOT:
                setShooter(velocityMap.get(getMeasuresToGoal(true)));
                setHood(hoodMap.get(getMeasuresToGoal(true)));
        }
    }
}

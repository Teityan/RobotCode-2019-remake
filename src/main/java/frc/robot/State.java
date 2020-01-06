package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class State {

    public enum DriveState {
        kManual,
        kLineTrace,
        kCloseToLine
    }

    public enum CargoState {
        kHold,
        kRelease,
        kDoNothing
    }

    public enum ClimbSequence {
        kDoNothing("kDoNothing"),
        kLiftUp("kLiftUp"),
        kLocking("kLocking"),
        kAdvance("kAdvance"),
        kUnlocking("kUnlocking"),
        kLiftDown("kLiftDown");

        String name;
        ClimbSequence(String name) {
            this.name = name;
        }
    }

    // Drive
    public DriveState driveState;
    public double driveStraightSpeed, driveRotateSpeed;    // コントローラー制御の値 
    public double driveStraightSetpoint, driveRotateSetpoint;    // PID制御の目標値
    public boolean is_drivePIDOn;    // PID制御するかどうか
    public boolean is_lineTraceOn;    // ライントレースするかどうか
    public boolean is_lowInputOn;    // 低出力モードにするかどうか

    // Lift
    public double liftSpeed;    // コントローラー制御の値
    public double liftSetpoint;   // PID制御の目標値
    public boolean is_liftPIDOn;    // PID制御するかどうか
    public boolean is_liftPIDOnTarget;	// 目標値に達したか

    // Grabber
    public CargoState cargoState;    // カーゴをどうするか  
    public boolean is_toHoldPanel;    // パネルを保持するかどうか
    public boolean is_toRetractArm;    //  アームをしまうかどうか
    public boolean is_holdingCargo;

    // Climb
    public ClimbSequence climbSequence = ClimbSequence.kDoNothing;    //    自動クライムの状態
    public boolean is_autoClimbOn;    // 自動クライムするかどうか
    public boolean is_lockingClimb = false;    // ストッパーを出すかどうか
    public boolean is_lockedClimb;    //ストッパーが完全に出されたかどうか
    public double climbMotorSpeed;    // クライムの時の後輪のモーターの値
    public boolean is_climbUp;
    public boolean is_startCounting;

    public State() {
        stateInit();
    }

    public void stateInit() {

        // Drive
        driveState = DriveState.kManual;
        driveStraightSpeed = 0;
        driveRotateSpeed = 0;
        is_drivePIDOn = false;
        is_lineTraceOn = false;
        is_lowInputOn = false;

        // Lift
        liftSpeed = 0;
        liftSetpoint = 0;
        is_liftPIDOn = false;

        // Grabber
        cargoState = CargoState.kDoNothing;
        is_toHoldPanel = true;
        is_toRetractArm = false;
        is_holdingCargo = false;
   
        // Climb
        is_autoClimbOn = false;
        climbMotorSpeed = 0;
        is_startCounting = false;
    }

    public void printVariables() {
        SmartDashboard.putNumber("ddriveStraightSpeed", driveStraightSpeed);
        SmartDashboard.putNumber("driveRotateSpeed", driveRotateSpeed);
        SmartDashboard.putNumber("ddriveStraightSetpoint", driveStraightSetpoint);
        SmartDashboard.putNumber("driveRotateSetpoint", driveRotateSetpoint);
        SmartDashboard.putNumber("liftSpeed", liftSpeed);
        SmartDashboard.putNumber("liftSetpoint", liftSetpoint);
        SmartDashboard.putNumber("climbMotorSpeed", climbMotorSpeed);

        SmartDashboard.putBoolean("is_drivePIDOn", is_drivePIDOn);
        SmartDashboard.putBoolean("is_lineTraceOn", is_lineTraceOn);
        SmartDashboard.putBoolean("is_liftPIDOn", is_liftPIDOn);
        SmartDashboard.putBoolean("is_toHoldPanel", is_toHoldPanel);
        SmartDashboard.putBoolean("is_toRetractArm", is_toRetractArm);
        SmartDashboard.putBoolean("is_autoClimbOn;", is_autoClimbOn);
        SmartDashboard.putBoolean("is_locking", is_lockingClimb);
        SmartDashboard.putBoolean("is_lockedClimb;", is_lockedClimb);

        SmartDashboard.putString("climbSequence", climbSequence.name);
    }

}

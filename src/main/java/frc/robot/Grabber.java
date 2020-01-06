/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
public class Grabber {

    private VictorSP rollerMotor;
    private Solenoid barSolenoid;
    private Solenoid armSolenoid;

    private boolean is_RollerMoving = false;
    private boolean is_retractingArm = true;
    private State state;
    
	/** private ShuffleboardTab tab= Shuffleboard.getTab("Drive");

	private NetworkTableEntry holdSpeed = tab.add("holdSpeed",0.3).getEntry();
	
    /**
		Ports
    */

    Grabber(VictorSP rollerMotor, Solenoid barSolenoid, Solenoid armSolenoid) {
        this.rollerMotor = rollerMotor;
        this.barSolenoid = barSolenoid;
        this.armSolenoid = armSolenoid;
        
    }
    
	public void applyState(State state) {
        this.state=state;
        
        if (state.is_toRetractArm || state.is_holdingCargo) {      
            // アームをしまう
            retractArm();
        } else {
            // アームを出す
            releaseArm();
        }

        
        switch (state.cargoState) {
            case kHold:
                if(state.is_holdingCargo){
                    
        if(state.is_lowInputOn){
            setRollerSpeed(0.1);
        }else{
        setRollerSpeed(1.0);   
        }
                } else {
                    holdCargo();
                }
                break;
            
            case kRelease:
                releaseCargo();
                break;

            case kDoNothing:
        
                if(is_retractingArm)  {
                    setRollerSpeed(0.3);
                }else {
                    stopRoller();
                }
                break;
            
            default:
                break;
        }

        if (state.is_toHoldPanel) {
            // パネルをつかむ
            holdPanel();
        } else {
            // パネルを離す
            releasePanel();
        }

        
    }
    
    private void holdCargo() {
        if(state.is_lowInputOn){
            setRollerSpeed(0.5);
        }else{
        setRollerSpeed(1.0);   
        }
        is_RollerMoving = true;
    }

    private void releaseCargo() {
        if(state.is_lowInputOn){
            setRollerSpeed(-0.5);
        }else{
        setRollerSpeed(-1.0);   
        }
        is_RollerMoving = true;
    }

    private void stopRoller() {
        rollerMotor.stopMotor();
        is_RollerMoving = false;
    }

    private void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
        is_RollerMoving = true;
    }

    public boolean is_RollerMoving() {
        return is_RollerMoving;
    }

    private void holdPanel() {
        barSolenoid.set(false);
    }

    private void releasePanel() {
        barSolenoid.set(true);   
    }

    private void retractArm() {
        armSolenoid.set(false); 
        is_retractingArm = true;  
    }

    private void releaseArm() {
        armSolenoid.set(true);    
        is_retractingArm = false;
    }

    public boolean is_retractingArm(){
        return is_retractingArm;
    }


    /*
    Grabber(Motor motor, Solenoid barSolenoid, Solenoid armSolenoid)
        モーターとソレノイドを受け取る

    applyState()
        Stateから情報を受け取り入力する

    holdCargo():
        モーターを回してCARGOを回収する
    releaseCargo();
        モーターを回してCARGOを射出する
    stopRoller();
        モーターを止める
    isRollerMoving()
        モーターが動いているか

    holdPanel();
        棒を広げてHATCH PANEL回収する
    releasePanl();
        棒を狭めてロボットが後退すればHATCH PANELを置けるようにする
    

    retractArm();
        アームをしまう
    releaseArm();
        アームを出す
        
    */

}

package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

public class Climb {

    private Talon climbMotor;
    private Solenoid climbSolenoid;

    Climb(Talon climbMotor, Solenoid climbSolenoid){
        this.climbMotor = climbMotor;
        this.climbSolenoid = climbSolenoid;
    }

    private void climbAdvance(double speed){
        climbMotor.setSpeed(speed);
    }

    private void climbLockStopper(){
        climbSolenoid.set(true);      
    }

    private void climbUnlockStopper(){
        climbSolenoid.set(false);
    }

    public void applyState(State state) {
        
       
        if(state.is_lockingClimb) {
            climbLockStopper();
        }else {
            climbUnlockStopper();
        }

        climbAdvance(state.climbMotorSpeed);
    
    }

}
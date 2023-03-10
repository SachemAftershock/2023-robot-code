package frc.lib;

/**
 * Linear Acceleration Mapping class
 * 
 * @author Arhum Mudassir
 */
public class AccelerationMapping {

    /**
     * Constructor for AccelerationMapping Class
     */
    public AccelerationMapping(){
    }
    
    /**
     * Applies a multiplier to reduce sharp changes in acceleration when going straight
     * 
     * @param pow the inputted power from the controller between -1.0 and 1.0
     * @param deadband 0.15 deadand
     */
    public static double linearShapeStraight(double pow, double deadband){
        //Uses a linear system to apply a multiplyer to the inputed throttle in order to 
        //dampen rapid changes in accleration

        double power = pow;
          
        //4 levels of power dampening applied based on inputed throttle
        //The values are not final and can and should be tuned to meet the requirements of the robot
        double highPowerDampening = 0.15;//0.25;
        double mediumHighPowerDampening = 0.20;
        double mediumPowerDampening = 0.25;//0.50;
        double mediumLowPowerDampening = 0.40;
        double lowPowerDampening = 0.65;//0.75;
        double noPowerDampening = 0.70;//1.0;
  
        //Ranges for inputed throttle
        //Can be tuned in conjunction to the power dampening levels to create smoother acceleration
        double stageOne = 0.30;
        double stageTwo = 0.50;
        double stageThree = 0.70;
        double stageFour = 0.80;
        double stageFive = 0.90;
        double stageSix = 1.0;
  
        if(power > 0.0){
            if(power < deadband) {return 0.0;}
            if(power <= stageOne && power > deadband) {return power*highPowerDampening;}
            if(power > stageOne && power <= stageTwo) {return power*mediumHighPowerDampening;}
            if(power > stageTwo && power <= stageThree) {return power*mediumPowerDampening;}
            if(power > stageThree || power <= stageFour) {return power*mediumLowPowerDampening;}
            if(power > stageFour || power <= stageFive) {return power*lowPowerDampening;}
            if(power > stageFive || power == stageSix) {return power*noPowerDampening;}
        }
          
        if(power < 0.0){
            if(power > -deadband) {return 0.0;}
            if(power >= -stageOne && power < -deadband) {return power*highPowerDampening;}
            if(power < -stageOne && power >= -stageTwo) {return power*mediumHighPowerDampening;}
            if(power < -stageTwo && power >= -stageThree) {return power*mediumPowerDampening; }
            if(power < -stageThree || power >= -stageFour) {return power*mediumLowPowerDampening;}
            if(power < -stageFour || power >= -stageFive) {return power*lowPowerDampening;}
            if(power < -stageFive || power == -stageSix) {return power*noPowerDampening;}
        }
        return 0;

    }

    /**
     * Applies a multiplier to reduce sharp changes in acceleration when turning
     * 
     * @param pow the inputted power from the controller between -1.0 and 1.0
     * @param deadband deadand is 0.15 to account for controller drift
     */
    public static double linearShapeTurn(double rot, double deadband){
        //Uses a linear system to apply a multiplyer to the inputed throttle in order to 
        //dampen rapid changes in accleration
  
        double power = rot;
          
        //6 levels of power dampening applied based on inputed throttle
        //The values are not final and can and should be tuned to meet the requirements of the robot
        double highPowerDampening = 0.15;//0.25;
        double mediumHighPowerDampening = 0.20;
        double mediumPowerDampening = 0.25;//0.50;
        double mediumLowPowerDampening = 0.40;
        double lowPowerDampening = 0.65;//0.75;
        double noPowerDampening = 0.70;//1.0;
  
        //Ranges for throttle dampening
        //Can be tuned in conjunction to the power dampening levels to create smoother acceleration
        double stageOne = 0.30;
        double stageTwo = 0.50;
        double stageThree = 0.70;
        double stageFour = 0.80;
        double stageFive = 0.90;
        double stageSix = 1.0;
  
        if(power > 0.0){
            if(power < deadband) {return 0.0;}
            if(power <= stageOne && power > deadband) {return power*highPowerDampening;}
            if(power > stageOne && power <= stageTwo) {return power*mediumHighPowerDampening;}
            if(power > stageTwo && power <= stageThree) {return power*mediumPowerDampening;}
            if(power > stageThree || power <= stageFour) {return power*mediumLowPowerDampening;}
            if(power > stageFour || power <= stageFive) {return power*lowPowerDampening;}
            if(power > stageFive || power == stageSix) {return power*noPowerDampening;}
        }
          
        if(power < 0.0){
            if(power > -deadband) {return 0.0;}
            if(power >= -stageOne && power < -deadband) {return power*highPowerDampening;}
            if(power < -stageOne && power >= -stageTwo) {return power*mediumHighPowerDampening;}
            if(power < -stageTwo && power >= -stageThree) {return power*mediumPowerDampening;}
            if(power < -stageThree || power >= -stageFour) {return power*mediumLowPowerDampening;}
            if(power < -stageFour || power >= -stageFive) {return -power*lowPowerDampening;}
            if(power < -stageFive || power == -stageSix) {return -power*noPowerDampening;}
        }
  
        return 0;
        
    }

}

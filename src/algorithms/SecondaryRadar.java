package algorithms;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;

public class SecondaryRadar extends Brain {
    //---VARIABLES---//
    private double distanceTravelled, distanceTravelledRocky, distanceTravelledRockyToScan;
    private int whoAmI;
    private double myX,myY;
    private double initX,initY,oldY,oldAngle;
    private boolean contourne,verslebas,wall;
    
    private static final int TEAM = 0xBADDAD;
    private static final int UNDEFINED = 0xBADC0DE0;
    private int state;
    private boolean inPosition, inTurnPoint;
    private static final int FIRE = 0xB52;
    private static final int FALLBACK = 0xFA11BAC;
    private static final int ROGER = 0x0C0C0C0C;
    private static final int OVER = 0xC00010FF;

    private static final int TURNLEFTTASK = 1;
    private static final int MOVETASK = 2;
    private static final int TOTURNPOINT = 4;
    private static final int TURNRIGHTTASK = 3;
    private static final int MOVEBACK = 5;

    private static final int SINK = 0xBADC0DE1;
    private static final double TARGET_DISTANCE = 700.0;
    private static final double TARGET_TURN_POINT = 300.0;
    private static final double HEADINGPRECISION = 0.1;
    private static final double ANGLEPRECISION = 0.01;
    private static final int ROCKY = 0x1EADDA;
    private static final int MARIO = 0x5EC0;

    private static final int DEPLACEMENT_CONTOURNER = 300;

    //---CONSTRUCTORS---//
    public SecondaryRadar() { super(); }

    @Override
    public void activate() {
        whoAmI = ROCKY;
        for (IRadarResult o: detectRadar()){ 
            if (o.getObjectType()==IRadarResult.Types.TeamSecondaryBot){
                if (isAbove(o.getObjectDirection(),Parameters.NORTH) && o.getObjectType()==IRadarResult.Types.TeamSecondaryBot){
                    whoAmI=MARIO;
                }
            }
        }

        if (whoAmI == ROCKY){
            myX=Parameters.teamASecondaryBot1InitX;
            myY=Parameters.teamASecondaryBot1InitY;
            initX=myX;
            initY=myY;

            state=TURNLEFTTASK;
        }else {
            myX=Parameters.teamASecondaryBot2InitX;
            myY=Parameters.teamASecondaryBot2InitY;
            state=TURNRIGHTTASK;
            initX=myX;
            initY=myY;
        }
        move();
        contourne= false;
        verslebas=false;
    }

    @Override
    public void step() {
  	  sendLogMessage( myX + " "+ myY);
    	
    /*
     * en face d'un allie, attends
     */
    if(detectFront().getObjectType()==IFrontSensorResult.Types.TeamSecondaryBot || detectFront().getObjectType()==IFrontSensorResult.Types.TeamMainBot){
        	 sendLogMessage("PTIN MAIS BOUGE");
   	  		return;
   	  }
    
      /*
	   * STEP : a côté d'un mur, pivote de 90°
	   */
	  if(wall) {
		  if(!(isSameDirection(getHeading(),oldAngle+Parameters.NORTH))){
			  stepTurn(Parameters.Direction.LEFT);
			  return;
		  }
	  }
	/*
     * STEP : Contournement d'un cadavre
     */
    
  	  if(contourne) {
  		  
  		  
		  /*
		   * pivote vers le bas 
		   */
			  if(verslebas == false &&!(isSameDirection(getHeading(),Parameters.SOUTH))) {
	              stepTurn(Parameters.Direction.RIGHT);
	              return;
			  }
              verslebas = true;		
              
              
    		  /*
    		   * se déplace vers le bas
    		   */
    			  
		      if(contourne && myY<oldY+DEPLACEMENT_CONTOURNER) {
				  move();
				  myY+=Parameters.teamASecondaryBotSpeed*Math.sin(getHeading());
		          return;
		      }
		      
    			  /*
        		   * pivote vers la gauche
        		   */
		      if(verslebas && !isSameDirection(getHeading(),Parameters.EAST)) {
	              stepTurn(Parameters.Direction.LEFT);
		          return;
			  }
    		  sendLogMessage("ESQUIVE");

		      contourne=false;
		      verslebas=false;
	  }
  	  
  	  
  	  /*
  	   * STEP : en face d'un mur, pivote de 90° 
  	   */
  	  if(detectFront().getObjectType()==IFrontSensorResult.Types.WALL){
  		wall=true;
    	 oldAngle = getHeading();

		return;
  	  }
  	  
  	  
  	  /*
       * STEP : detection d'un object
       */
        if (state==MOVETASK){
            //sendLogMessage("In position. Ready to fire! " + (name));
            for (IRadarResult o: detectRadar()){
            	/*
            	 * Detection d'un ennemis, envoie des coordonée
            	 */
                if (o.getObjectType()==IRadarResult.Types.OpponentMainBot || o.getObjectType()==IRadarResult.Types.OpponentSecondaryBot) {
                	sendLogMessage("DETECTE");
                	double enemyX=0;
                	double enemyY=0;
                    enemyX=myX+o.getObjectDistance()*Math.cos(o.getObjectDirection());
                    enemyY=myY+o.getObjectDistance()*Math.sin(o.getObjectDirection());
                    //sendLogMessage((int)myX +" "+ (int)myY+ " "+(int)enemyX + " "+ (int)enemyY);
                    broadcast(enemyX+":"+enemyY);
                	inPosition = true;
                	return;
                	
                	
                	/*
                	 * Detection d'un cadavre, on esquive
                	 */
                }else if(o.getObjectType()==IRadarResult.Types.Wreck){
                	if(o.getObjectDistance()<150) {
                  	  if(!contourne) {
                  		  sendLogMessage("contourne");
                  		  oldY=myY;
                  		  contourne = true;
                  	  
                  	  }
                	}
                	broadcast("-1:-1");
            		  break;
                	
                }else {
                	/*
                	 * Detection d'un ennemis, envoie des coordonée
                	 */
                	sendLogMessage("je vois personne");
                	inPosition = false;
                }
            }

        }
  
       
        
        
    	/* ------------------------	les instructions de ROCKY  ------------------------ */
        if (whoAmI== ROCKY){
        	
        	/*
        	 * STEP : il pivote
        	 */
            if (state==TURNLEFTTASK) {
                if (!(isSameDirection(getHeading(),Parameters.NORTH))) {
                    //System.err.println("I am Rocky and I am turning left");
                    stepTurn(Parameters.Direction.LEFT);
                    return;
                }else {
                    state=TOTURNPOINT;
                }
            }
            
            
            /*
             * STEP : il monte
             */
            if (state==TOTURNPOINT && !(isHeading(Parameters.EAST))) {
                if (myY > initY-TARGET_TURN_POINT) {
                    move();
                    myY+=Parameters.teamASecondaryBotSpeed*Math.sin(getHeading());
                    return;
                }
                stepTurn(Parameters.Direction.RIGHT);
                return;
            } else {
                inTurnPoint = true;
            }
            
            
            /*
             * STEP : atteint sa destination : changement d'état
             */
            if(inTurnPoint && (state!=MOVETASK)){
                state=MOVETASK;
                return;
            }

            /*
             * STEP: il se déplace de gauche à droite
             */
            if (state==MOVETASK && !inPosition) {
            	myX +=Parameters.teamBSecondaryBotSpeed*Math.cos(getHeading());
                move();
                return;
            } else {
            	inPosition = true;
                
            }
            
            
            /*
             * STEP: en face d'un mur
             */

            if(state==MOVEBACK) {
            	sendLogMessage("WALL");
            	myX -=Parameters.teamBSecondaryBotSpeed*Math.cos(getHeading());
            	moveBack();
            	return;
            }else {
            	inPosition = true;
            }
    /* ------------------------	les instructions de MARIO  ------------------------ */

        }else {
        	
        	/*
        	 * STEP : il pivote
        	 */
        	if (state==TURNRIGHTTASK) {
                if (!(isSameDirection(getHeading(),Parameters.SOUTH))) {
                    stepTurn(Parameters.Direction.RIGHT);
                    return;
                }else {
                    state=TOTURNPOINT;
                }
            }

        	
        	/*
        	 * STEP : il descend
        	 */
            if (state==TOTURNPOINT && !(isHeading(Parameters.EAST))) {
                if (myY < initY+TARGET_TURN_POINT) {
                    move();
                    myY+=Parameters.teamASecondaryBotSpeed*Math.sin(getHeading());
                    distanceTravelledRocky += Parameters.teamBSecondaryBotSpeed;
                    return;
                }
                stepTurn(Parameters.Direction.LEFT);
                return;
            } else {
                inTurnPoint = true;
            }

            
            /*
             * STEP : atteint sa destination : changement d'état
             */
            if(inTurnPoint && (state!=MOVETASK)){
                state=MOVETASK;
                return;
            }

            
            /*
             * STEP : se déplace de gauche à droite 
             */
            if (state==MOVETASK && !inPosition ) {
            	move();
            	myX +=Parameters.teamBSecondaryBotSpeed*Math.cos(getHeading());                    return;
            } else {
                inPosition = true;
            }
            
            
            /*
             * STEP: en face d'un mur
             */

            if(state==MOVEBACK) {
            	sendLogMessage("WALL");
            	myX -=Parameters.teamBSecondaryBotSpeed*Math.cos(getHeading());
            	moveBack();
            	return;
            }else {
            	inPosition = true;
            }
            
        }
        

        //System.out.println("---------------------------------------------------------------------");
    }

    private boolean isHeading(double dir){
    	//System.out.println(Math.abs(Math.sin(getHeading()-dir)));
        return Math.abs(Math.sin(getHeading()-dir))<ANGLEPRECISION;
    }

    private double myGetHeading(){
        double result = getHeading();
        while(result<0) result+=2*Math.PI;
        while(result>2*Math.PI) result-=2*Math.PI;
        return result;
    }
    private boolean isSameDirection(double dir1, double dir2){
        return Math.abs(dir1-dir2)<ANGLEPRECISION;
    }

    private double normalize(double dir){
        double res=dir;
        while (res<0) res+=2*Math.PI;
        while (res>=2*Math.PI) res-=2*Math.PI;
        return res;
    }

    private boolean isAbove(double dir1, double dir2){
        return Math.abs(normalize(dir1)-normalize(dir2))<HEADINGPRECISION;
    }
    
    private void contourner() {
    	
    }
}

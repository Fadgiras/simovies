/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * Copyright (C) 2014 <Binh-Minh.Bui-Xuan@ens-lyon.org>.
 * GPL version>=3 <http://www.gnu.org/licenses/>.
 * $Id: algorithms/Combinaison.java 2014-10-19 buixuan.
 * ******************************************************/
package algorithms;

import java.util.ArrayList;

import robotsimulator.Brain;
import characteristics.Parameters;
import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;

public class Combinaison extends Brain {
  //---PARAMETERS---//
  private static final double HEADINGPRECISION = 0.01;
  private static final double ANGLEPRECISION = 0.001;
  
  private static final int WEEB = 0xA1EADDA;
  private static final int DJIDJI = 0xB5EC0;
  private static final int BLEP = 0xCBADDADD;

  private static final int DEPLACEMENT = 300;
  private static final int DISTANCE_MAX = 1000;
  private static final int DEPLACEMENT_CONTOURNER = 300;


  private Parameters.Direction turnDirectionForAvoiding;

  private int whoAmI;

  //---VARIABLES---//
  private boolean turnRightTask,fallBackCoveringFireTask, avoidTeambot, avoidingTeammate,avoidingWreck;
  private double endTaskDirection,endMoveTask,distance;
  private static enum Action { NONE, MOVE, MOVEBACK, FIRELEFT, FIRERIGHT, FIRE };
  private static Action[] fallBackCoveringFireScheme = { Action.FIRE, Action.MOVEBACK, Action.FIRELEFT, Action.MOVEBACK, Action.FIRERIGHT, Action.MOVEBACK };
  //private static Action[] fallBackCoveringFireScheme = { Action.FIRE, Action.MOVEBACK};
  private int schemeIndex;
  private double cibleX,cibleY,cibleAngle;
  private double oldCibleX,oldCibleY;
  private double oldY;
  private double myX,myY;
  private double initX;
  private double oldAngle;
  private ArrayList<String> messages;
  private ArrayList<IRadarResult> listRadar;
  private boolean messagerecu,enemy,contourne,verslebas,wall;
  public Combinaison() { super(); }

  public void activate() {
	  /*
	   * Assigne les différents roles
	   */
	    listRadar = detectRadar();
	    if(isAbove(listRadar.get(0).getObjectDirection(),Parameters.SOUTH,HEADINGPRECISION) && isAbove(listRadar.get(1).getObjectDirection(),Parameters.EAST,HEADINGPRECISION)){
	    	whoAmI = WEEB;
	    }
	    if(isAbove(listRadar.get(0).getObjectDirection(),Parameters.NORTH,HEADINGPRECISION) && isAbove(listRadar.get(1).getObjectDirection(),Parameters.SOUTH,HEADINGPRECISION)){
	    	whoAmI = DJIDJI;
	    }
	    if(isAbove(listRadar.get(0).getObjectDirection(),Parameters.NORTH,HEADINGPRECISION) && isAbove(listRadar.get(1).getObjectDirection(),Parameters.EAST,HEADINGPRECISION)){
	    	whoAmI = BLEP;
	    }
	    if (whoAmI == WEEB){
	      myX=Parameters.teamAMainBot1InitX;
	      myY=Parameters.teamAMainBot1InitY;
	    }
	    if(whoAmI == DJIDJI) {
	      myX=Parameters.teamAMainBot2InitX;
	      myY=Parameters.teamAMainBot2InitY;
	    }
	    if(whoAmI == BLEP) {
	    	myX=Parameters.teamAMainBot3InitX;
		    myY=Parameters.teamAMainBot3InitY;
	    }
	    initX=myX;
	    turnRightTask=false;
	    enemy=false;
		messagerecu = false;
		oldCibleX=-1;
		oldCibleY=-1;
		contourne=false;
		verslebas=false;
		wall=false;

  }
  
  public void step() {
	  sendLogMessage( myX + " "+ myY);
	  
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
	   * STEP : devant lui un allié, attends
	   */
	  if(detectFront().getObjectType()==IFrontSensorResult.Types.TeamSecondaryBot || detectFront().getObjectType()==IFrontSensorResult.Types.TeamMainBot){
     	 sendLogMessage("PTIN MAIS BOUGE");
	  		return;
	  	}
	  
	  
	  if(contourne) {
		  /*
		   * si on detecte un object au dessus de lui
		   */
			  if(verslebas == false &&!(isSameDirection(getHeading(),Parameters.SOUTH))) {
	              stepTurn(Parameters.Direction.RIGHT);
	              return;
			  }
              verslebas = true;			  
		      if(myY<oldY+DEPLACEMENT_CONTOURNER) {
				  move();
				  myY+=Parameters.teamASecondaryBotSpeed*Math.sin(getHeading());
		          return;
		      }
		      
		      if(verslebas && !isSameDirection(getHeading(),Parameters.EAST)) {
	              stepTurn(Parameters.Direction.LEFT);
		          return;
			  }

		      contourne=false;
		      verslebas=false;
	  }
	  
	  
	  /*
	   * en face d'un cadavre, contourne
	   
	  if(detectFront().getObjectType()== IFrontSensorResult.Types.Wreck){
		  contourne = true;
		  return;
		}
	  */
	  
	  /*
	   * detection d'un mur 
	   */
	  if(detectFront().getObjectType()==IFrontSensorResult.Types.WALL){
     	 wall = true;
     	 oldAngle = getHeading();
     	 return;
	  }
	  
	  /*
	   * a porté de radar, tire sur les ennemis 
	   * 
	   * si il detecte un cadavre sur son radar, il va l'esquive
	   */
      for (IRadarResult o: detectRadar()){
          if (o.getObjectType()==IRadarResult.Types.OpponentMainBot || o.getObjectType()==IRadarResult.Types.OpponentSecondaryBot) {
        	  fire(o.getObjectDirection());
        	  return;
          }
          else if(o.getObjectType()==IRadarResult.Types.Wreck){
        	  if(o.getObjectDistance()<150) {
            	  if(!contourne) {
            		  sendLogMessage("contourne");
            		  oldY=myY;
            		  contourne = true;
            		  break;
            	  
            	  }
        	  }
         
          }

      }
      
      

	  /*
	   * STEP : Reception des coordonnées de l'ennemie
	   */
	  messages= fetchAllMessages();
	  if(!messages.isEmpty()) { 
			  //System.out.println("Message recu "+ cibleX +" " +cibleY);
			  handleMessages(messages);
			  messages.clear();
			
	  }
	 
	  /*
	   * STEP : Tire si il est à porté 
	   */
	  
	  
	  if(messagerecu) {
		 if(isDistanceInf(myX,myY,cibleX,cibleY,DISTANCE_MAX)) {
			 cibleAngle = tournerVers(myX,myY,cibleX,cibleY);
			 //sendLogMessage(cibleAngle+" "+(int)myX + " "+ (int)myY+ " "+(int)(oldCibleX- myX) + " "+ (int)(oldCibleY- myY));
			 fire(cibleAngle);
			 return;
		 }else {
			 myMove();
		 }
	  }
	
  }
  
  
  
  /*-------------------FONCTION PRIVEE-------------------*/
	private double distanceEuclidienne(double x1,double y1, double x2,double y2) {
		return Math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
	}
	
	private boolean isDistanceInf(double x1,double y1, double x2,double y2,double dm) {
		return distanceEuclidienne(x1,y1,x2,y2) < dm;
	}
	
	private double tournerVers(double x, double y, double destX, double destY) {
	    return Math.atan2(destY - y, destX - x);
	}
	
	private double normalize(double dir){
	    double res=dir;
	    while (res<0) res+=2*Math.PI;
	    while (res>=2*Math.PI) res-=2*Math.PI;
	    return res;
	}
	
  private boolean isHeading(double dir){
    return Math.abs(Math.sin(myGetHeading()-dir))<HEADINGPRECISION;
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
  private boolean isAbove(double dir1, double dir2,double precision){
	  //System.out.println(normalize(dir1)-normalize(dir2));
      return Math.abs(normalize(dir1)-normalize(dir2))<HEADINGPRECISION;
  }
  
  private void handleMessages(ArrayList<String> messages) {
		  for (String message : messages) {
	          String[] parts = message.split(":");
	          cibleX = Double.parseDouble(parts[0]);
	          cibleY= Double.parseDouble(parts[1]);
	          messagerecu = true;
	      }	  
	  
      
  }
  
  private void myMove() {
	  move();
	  myX+=Parameters.teamAMainBotSpeed*Math.cos(getHeading());
      myY+=Parameters.teamAMainBotSpeed*Math.sin(getHeading());
	  
  }
  
}

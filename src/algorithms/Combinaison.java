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

  private static final int DEPLACEMENT = 500;

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
  private double myX,myY;
  private double initX;
  private ArrayList<String> messages;
  private ArrayList<IRadarResult> listRadar;
  private boolean messagerecu,enemy;
  public Combinaison() { super(); }

  public void activate() {
	    
	    listRadar = detectRadar();
	    if(isAbove(listRadar.get(0).getObjectDirection(),Parameters.SOUTH) && isAbove(listRadar.get(1).getObjectDirection(),Parameters.EAST)){
	    	whoAmI = WEEB;
	    }
	    if(isAbove(listRadar.get(0).getObjectDirection(),Parameters.NORTH) && isAbove(listRadar.get(1).getObjectDirection(),Parameters.SOUTH)){
	    	whoAmI = DJIDJI;
	    }
	    if(isAbove(listRadar.get(0).getObjectDirection(),Parameters.NORTH) && isAbove(listRadar.get(1).getObjectDirection(),Parameters.EAST)){
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

  }
  
  public void step() {
	  sendLogMessage( myX + " "+ myY);
	  
	  /*
	   * STEP : Reception des coordonnées de l'ennemie
	   */
	  messages= fetchAllMessages();
	  if(!messages.isEmpty()) { 
		  if(oldCibleX == -1 && oldCibleY == -1) {
			  handleMessages(messages);
			  oldCibleX= cibleX;
			  oldCibleY= cibleY;
		  }else {
			  //System.out.println("Message recu "+ cibleX +" " +cibleY);
			  handleMessages(messages);
			  if(distanceEuclidienne(myX,myY,cibleX,cibleY) < distanceEuclidienne(myX,myY,oldCibleX,oldCibleY)) {
				  oldCibleX = cibleX;
				  oldCibleY = cibleY;
			  }
			  messages.clear();
			  messagerecu = true;  
		  }
			
	  }else {
			messagerecu = false;
	  }
	 
	  /*
	   * STEP : Tire si il est à porté 
	   */
	  
	  
	  if(messagerecu) {
		 if(isDistanceInf(myX,myY,oldCibleX,oldCibleY)) {
			 cibleAngle = tournerVers(myX,myY,oldCibleX,oldCibleY);
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
	
	private boolean isDistanceInf(double x1,double y1, double x2,double y2) {
		return distanceEuclidienne(x1,y1,x2,y2) < 600.0;
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
  private boolean isAbove(double dir1, double dir2){
      return Math.abs(normalize(dir1)-normalize(dir2))<HEADINGPRECISION;
  }
  private void handleMessages(ArrayList<String> messages) {
      for (String message : messages) {
          String[] parts = message.split(":");
          cibleX = Double.parseDouble(parts[0]);
          cibleY= Double.parseDouble(parts[1]);
      }
  }
  
  private void myMove() {
	  move();
	  myX+=Parameters.teamAMainBotSpeed*Math.cos(getHeading());
      myY+=Parameters.teamAMainBotSpeed*Math.sin(getHeading());
	  
  }
}

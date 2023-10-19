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

  private static final int DEPLACEMENT = 200;

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
  private double myX,myY;
  private ArrayList<String> messages;
  private ArrayList<IRadarResult> listRadar;
  private int compteur;
  private boolean messagerecu;
  //---CONSTRUCTORS---//
  public Combinaison() { super(); }

  //---ABSTRACT-METHODS-IMPLEMENTATION---//
  public void activate() {
	  
	  //ODOMETRY CODE
	    
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
	    turnRightTask=false;
	    move();
	    
  }
  
  public void step() {
	  sendLogMessage("position "+ myX + " "+ myY);
	  compteur++;

	  //reception de message 
	  messages= fetchAllMessages();
	  if(!messages.isEmpty()) {
			sendLogMessage("Message recu "+ cibleX +" " +cibleY);
			handleMessages(messages);
			messages.clear();
			messagerecu = true;
	  }
	  if(messagerecu) {
		  if(isDistanceInf(myX,myY,cibleX,cibleY)) {
				cibleAngle=tournerVers(myX,myY,cibleX,cibleY);
				fire(cibleAngle);
			}
	  }
	  
	  if(compteur < DEPLACEMENT) {
		  move();
		  myX+=Parameters.teamASecondaryBotSpeed*Math.cos(getHeading());
	      myY+=Parameters.teamASecondaryBotSpeed*Math.sin(getHeading());
		  compteur++;
	  }

    return;
  }
  
  /*-------------------FONCTION PRIVEE-------------------*/
	private double distanceEuclidienne(double x1,double y1, double x2,double y2) {
		return Math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
	}
	
	private boolean isDistanceInf(double x1,double y1, double x2,double y2) {
		return distanceEuclidienne(x1,y1,x2,y2) < 1000.0;
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
    return Math.abs(Math.sin(getHeading()-dir))<HEADINGPRECISION;
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
}

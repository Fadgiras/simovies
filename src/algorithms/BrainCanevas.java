/* ******************************************************
 * Simovies - Eurobot 2015 Robomovies Simulator.
 * Copyright (C) 2014 <Binh-Minh.Bui-Xuan@ens-lyon.org>.
 * GPL version>=3 <http://www.gnu.org/licenses/>.
 * $Id: algorithms/BrainCanevas.java 2014-10-19 buixuan.
 * ******************************************************/
package algorithms;

import robotsimulator.Brain;
import characteristics.IFrontSensorResult;
import characteristics.Parameters;

public class BrainCanevas extends Brain {
	private double myX,myY;
  public BrainCanevas() { super(); }

  public void activate() {
    //---PARTIE A MODIFIER/ECRIRE---//
    move();
    myX = 2500;
    myY = 800;
  }
  public void step() {
    //---PARTIE A MODIFIER/ECRIRE---//
	sendLogMessage("Position : "+ myX + " " + myY );
    if (detectFront().getObjectType()==IFrontSensorResult.Types.NOTHING) {
      move();
      myX +=Parameters.teamBSecondaryBotSpeed*Math.cos(getHeading());
      myY +=Parameters.teamBSecondaryBotSpeed*Math.sin(getHeading());
    }
  }
}

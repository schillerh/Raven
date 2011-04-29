package raven.game.navigation;

import java.util.ArrayList;

import java.util.Iterator;
import java.util.List;

import raven.game.RavenBot;
import raven.game.messaging.Dispatcher;
import raven.game.navigation.RavenPathPlanner;
import raven.game.navigation.*;
import raven.game.navigation.NavGraphEdge;
import raven.game.navigation.NavGraphNode;


public class PathManager<RavenPathPlanner> {
	private ArrayList<RavenPathPlanner> searchRequests;
	private int numSearchCyclesPerUpdate;
	public PathManager(int numCyclesPerUpdate) {
		// TODO Auto-generated constructor stub
	}

	public void Register(RavenPathPlanner pathPlanner){
		//make sure the bot does not already have a current search in the queue
		if(searchRequests.contains(pathPlanner)) return;
		else searchRequests.add(pathPlanner);

	}

	public void UnRegister(RavenPathPlanner pathPlanner){
		searchRequests.remove(pathPlanner);
	}

	//returns the amount of path requests currently active.
	public int  GetNumActiveSearches(){return searchRequests.size();}

	///////////////////////////////////////////////////////////////////////////////
	//------------------------- UpdateSearches ------------------------------------
	//
	//  This method iterates through all the active path planning requests 
	//  updating their searches until the user specified total number of search
	//  cycles has been satisfied.
	//
	//  If a path is found or the search is unsuccessful the relevant agent is
	//  notified accordingly by Telegram
	//-----------------------------------------------------------------------------
	public void updateSearches()
	{
		int NumCyclesRemaining = numSearchCyclesPerUpdate;

		//iterate through the search requests until either all requests have been
		//fulfilled or there are no search cycles remaining for this update-step.

		for(int i=0; NumCyclesRemaining-->0 && !searchRequests.isEmpty(); i++)
		{	
			RavenPathPlanner curSearch;

			curSearch = searchRequests.get(i);
			int result = ((GraphSearchAStar)curSearch).search();
			if(result == 1 ||result == 0){
				searchRequests.remove(i);}


		}//end while
	}



}

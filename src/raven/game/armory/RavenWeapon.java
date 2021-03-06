/**
 * 
 */
package raven.game.armory;

import java.util.ArrayList;
import java.util.List;

import com.sun.servicetag.SystemEnvironment;

import raven.game.RavenBot;
import raven.game.RavenObject;
import raven.goals.fuzzy.FuzzyModule;
import raven.math.Vector2D;
import raven.utils.Log;

/**
 * @author chester
 *
 */
public abstract class RavenWeapon {

	private RavenBot owner;
	private RavenObject itemType;
	private FuzzyModule fuzzyModule;
	private int roundsLeft, maxRoundCapacity; 
	private double rateOfFire, lastDesireabilityScore, idealRange, maxProjectileSpeed;
	public double timeUntilAvailable;
	private List<Vector2D> WeaponVB, WeaponVBTrans;
	
	public RavenWeapon(RavenObject weaponType, int defaultRoundsCount, int maxCapacity, 
					   double RoF, double iRange, double projectileSpd, RavenBot holder)
	{
		itemType = weaponType;
		owner = holder;
		roundsLeft = defaultRoundsCount;
		maxRoundCapacity = maxCapacity;
		rateOfFire = RoF;
		timeUntilAvailable = 0;
		idealRange = iRange;
		maxProjectileSpeed = projectileSpd;
		lastDesireabilityScore = 0;
		
		fuzzyModule = new FuzzyModule();
		WeaponVB = new ArrayList<Vector2D>();
		WeaponVBTrans = new ArrayList<Vector2D>();
	}
	
	
	/* OVERRIDES */
	
	/**
	 * Causes the weapon to shoot at the chosen position. Each weapons overrides this method.
	 * @param position
	 */
	public void ShootAt(Vector2D position) {}
	
	/**
	 * Draws the weapon on the display via GameCanvas static calls.  Overridden by each weapon.
	 */
	public void render() {}
	
	/**
	 * This overridden method uses fuzzy logic to assign a desireability value to this weapon, based on the distance and the logic
	 * currently assigned to the weapon.
	 * @param distanceToTarget
	 * @return
	 */
	public double GetDesireability(double distanceToTarget) { return 0; }
	
	/* ACCESSORS */
	
//	public final boolean AimAt(Vector2D target) { return owner.rotateFacingTowardPosition(target); }
	
	public double getMaxProjectileSpeed() { return maxProjectileSpeed; }
	
	public int getRoundsRemaining() { return roundsLeft; }
	
	public void decrementRoundsLeft() { if(roundsLeft > 0) --roundsLeft; }
	
	public List<Vector2D> getWeaponVectorBuffer() { return WeaponVB; }
	
	public List<Vector2D> getWeaponVectorTransBuffer() { return WeaponVBTrans; }
	
	public void setWeaponVectorTransBuffer(List<Vector2D> tempBuffer) { WeaponVBTrans = tempBuffer; }
	
	public RavenBot getOwner() { return owner; }
	
	public FuzzyModule getFuzzyModule() { return fuzzyModule; }
	
	public void setLastDesireability(double newDesireability) { lastDesireabilityScore = newDesireability; }
	/**
	 * Adds the specified number of rounds to this weapons' magazine, without exceeding the max capacity.
	 * @param numberToAdd
	 */
	public void incrementRounds(int numberToAdd){
		roundsLeft += numberToAdd; 
		if(roundsLeft > maxRoundCapacity) roundsLeft = maxRoundCapacity;
	}
	
	public RavenObject getWeaponType() {return itemType; }
	
	public double getIdealRange() { return idealRange; }
	
	
	
	
	public boolean isReadyForNextShot() { 
		if(System.currentTimeMillis()>timeUntilAvailable) {
			Log.debug("RavenWeapon", "isReadyForNextShot:timeUntilAvailable: "+timeUntilAvailable+"millis time: "+System.currentTimeMillis());	
				
			return true;
		}
		return false; 
	}
	
	public void UpdateTimeWeaponIsNextAvailable(double delta) { 
		
		timeUntilAvailable = System.currentTimeMillis()+1000*(1.0/delta); 
		Log.debug("RavenWeapon", "UpdateTimeWeaponIsNextAvailable: delta:"+delta+"timeUntilAvailable: "+timeUntilAvailable);	
		return;
	}
	
	@SuppressWarnings("unused")
	private void InitializeFuzzyModule() {}


	public double getLastDesirabilityScore() {
		return lastDesireabilityScore;
	}
	
	public int getMaxRounds()
	{
		return maxRoundCapacity;
	}
	
	
}

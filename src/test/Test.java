package test;

import cs.util.Simulate;

public class Test {
	public int[] worstDistInTime = new int[] { 0, 0, 0,
	    3,  3,  7, 12, 12, 19, 26, 26, 35, 46, 54, 62, 69, 76, 84, 91,
	   98,105,112,119,125,132,138,144,150,156,162,167,173,181,188,195,
	  203,210,218,226,234,242,250,258,266,274,282,290,298,306,314,322,
	  330,338,346,354,362,370,378,386,394,402
	};
	
	/**
	 * @param args
	 */
	public static void test(String[] args) {
		//how far can we move in X
		for(int i=3;i<=60;++i) {
			Simulate sim = new Simulate();
			
			sim.velocity = 0;
			sim.direction = 1;
			sim.angleToTurn = Math.PI;
			
			boolean turn = false;
			int time = i;
			//accelerate
			while(timeToStop(sim.velocity) < time--) {
				if(!turn && sim.angleToTurn == 0) {
					sim.angleToTurn -= Math.PI/2.0;
					turn = true;
				}
				sim.step();
			}
			
			//try and slow down and stop
			sim.direction = 0;
			
			for(int t=0;t<timeToStop(sim.velocity);++t) {
				--time;
				sim.step();
			}
			
			System.out.printf("%3.0f\n",sim.position.length());
			
			//System.out.println(time);
		}
		
	}
	
	public static int timeToSpeedFromStop(double velocity) {
		if(velocity < 0)
			velocity = -velocity;
		if((int)velocity == velocity)
			return (int)velocity;
		return (int)velocity+1;
	}
	
	
	public static int timeToStop(double velocity) {
		velocity = Math.abs(velocity);
		if(velocity >= 6)
			return 4;
		if(velocity >= 4)
			return 3;
		if(velocity >= 2)
			return 2;
		if(velocity > 0)
			return 1;
		return 0;
	}
}

#ifndef PARTICLE_H
#define PARTICLE_H

class Particle
{
	private:
		int x;
		int y;
		float theta;
		float weight;
	
	public:
		Particle();
		void fitness();
};

#endif

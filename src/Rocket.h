#ifndef ROCKET_H_
#define ROCKET_H_

#include "Project.h"

namespace T4T {

class Rocket : public Project
{
public:
	class Straw : public Project::Element {
		public:
		PhysicsGenericConstraint *_constraint;

		Straw(Project *project);
		void addPhysics(short n);
		bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	};
	
	class Balloon : public Project::Element {
		public:
		std::vector<float> _balloonRadius, _anchorRadius;

		Balloon(Project *project, Element *parent);
		void placeNode(short n);
		void addPhysics(short n);
	};

	Straw *_straw;
	Balloon *_balloons;
	float _strawRadius, _strawLength, _originalStrawLength, _pathLength;
	bool _deflating;

	Rocket();
	void sync();
	void setActive(bool active);
	bool setSubMode(short mode);
	bool positionPayload();
	bool removePayload();
	void launch();
	void update();
	void launchComplete();
	void controlEvent(Control *control, Control::Listener::EventType evt);
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
};

}

#endif

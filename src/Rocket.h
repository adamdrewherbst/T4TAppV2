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
	float _strawRadius, _strawLength, _originalStrawLength;
	bool _launching;

	Rocket();
	void setActive(bool active);
	bool setSubMode(short mode);
	void launch();
	void update();
	void controlEvent(Control *control, EventType evt);
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
};

}

#endif

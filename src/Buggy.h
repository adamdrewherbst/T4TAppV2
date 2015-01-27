#ifndef BUGGY_H_
#define BUGGY_H_

#include "Project.h"

namespace T4T {

class Buggy : public Project {
public:
	class Body : public Project::Element {
		public:
		Body(Project *project);
	};

	class Axle : public Project::Element {
		public:
		Axle(Project *project, Element *parent, const char *id, const char *name);
		void placeNode(short n);
		void addPhysics(short n);
	};

	class Wheels : public Project::Element {
		public:
		Wheels(Project *project, Element *parent, const char *id, const char *name);
		void placeNode(short n);
		void addPhysics(short n);
	};
	
	Element *_body, *_frontAxle, *_rearAxle, *_frontWheels, *_rearWheels;
	MyNode *_ramp;
	float _rampSlope;

	Buggy();
	void setupMenu();
	void setActive(bool active);
	bool setSubMode(short mode);
	void launch();
	void controlEvent(Control *control, Control::Listener::EventType evt);
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	void setRampHeight(float scale);
};

}

#endif

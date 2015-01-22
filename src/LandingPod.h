#ifndef LANDINGPOD_H_
#define LANDINGPOD_H_

#include "Project.h"

namespace T4T {

class LandingPod : public Project {
public:
	class Body : public Project::Element {
		public:
		ConstraintPtr _groundAnchor;
		
		Body(Project *project);
	};

	class Hatch : public Project::Element {
		public:
		ConstraintPtr _lock;
		
		Hatch(Project *project, Element *parent);
		void placeNode(short n);
		void addPhysics(short n);
	};
	
	Body *_body;
	Hatch *_hatch;
	
	MyNode *_buggy; //use the lunar buggy as the payload
	
	Button *_hatchButton;

	LandingPod();
	bool touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);
	void controlEvent(Control *control, EventType evt);
	void setupMenu();
	void setActive(bool active);
	bool setSubMode(short mode);
	void launch();
	void openHatch();
};

}

#endif

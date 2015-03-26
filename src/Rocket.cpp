#include "T4TApp.h"
#include "Rocket.h"
#include "MyNode.h"

namespace T4T {

Rocket::Rocket() : Project::Project("rocket") {

	app->addItem("straw", 2, "general", "straw");
	app->addItem("balloon_sphere", 2, "general", "balloon");
	app->addItem("balloon_long", 2, "general", "balloon");
	
	_payloadId = "satellite";

	_pathLength = 10;
	_straw = (Straw*) addElement(new Straw(this));
	_balloons = (Balloon*) addElement(new Balloon(this, _straw));
	setupMenu();
}

void Rocket::setActive(bool active) {
	Project::setActive(active);
}

bool Rocket::setSubMode(short mode) {
	bool changed = Project::setSubMode(mode);
	_launching = false;
	switch(mode) {
		case 0: { //build
			break;
		} case 1: { //test
			//move the straw back to the starting point
			Vector3 start(0, 0, -_pathLength/2);
			app->setCameraEye(30, 0, M_PI/12);
			app->getPhysicsController()->setGravity(app->_gravity);
			_straw->_constraint->setEnabled(false);
			_rootNode->enablePhysics(false);
			_rootNode->setMyTranslation(start);
			_rootNode->enablePhysics(true);
			_straw->_constraint->setEnabled(true);
			//dangle the lunar buggy from the center of the straw with a socket constraint
			positionPayload();
			/*for(short i = 0; i < 2; i++) {
				Vector3 springJoint(joint.x, joint.y, joint.z + (2*i-1) * _strawLength/3);
				PhysicsSpringConstraint *constraint = 
					(PhysicsSpringConstraint*) app->addConstraint(straw, body, -1, "spring", springJoint, dir, true);
				constraint->setLinearStrengthZ(0.1f);
			}*/
			break;
		}
	}
	return changed;
}

bool Rocket::positionPayload() {
	if(!Project::positionPayload()) return false;
	MyNode *straw = _straw->getNode();
	straw->updateTransform();
	BoundingBox strawBox = straw->getBoundingBox(true);
	BoundingBox satelliteBox = _payload->getBoundingBox(true);
	Vector3 trans = straw->getTranslationWorld();
	trans.y += strawBox.min.y - satelliteBox.max.y - 1.5f;
	_payload->setMyTranslation(trans);
	_payload->enablePhysics(true);
	if(_payload->getScene() != _scene) {
		Vector3 joint = trans, dir = Vector3::unitY();
		joint.y += satelliteBox.max.y;
		app->addConstraint(straw, _payload, -1, "fixed", joint, dir, true);
	}
	return true;
}

bool Rocket::removePayload() {
	MyNode *straw = _straw->getNode();
	if(_payload) app->removeConstraints(straw, _payload, true);
	return Project::removePayload();
}

void Rocket::launch() {
	Project::launch();
	_deflating = true;
}

void Rocket::update() {
	if(!_launching) return;
	//see if we made it to the end
	float maxZ = _rootNode->getFirstChild()->getTranslationWorld().z; //getMaxValue(Vector3::unitZ());
	if(maxZ > 0.99f * _pathLength/2) {
		app->message("You made it to the end!");
	}
	//deflate each balloon by a fixed percentage
	if(!_deflating) return;
	_deflating = false;
	short n = _balloons->_nodes.size();
	MyNode *straw = _straw->getNode();
	for(short i = 0; i < n; i++) {
		MyNode *balloon = _balloons->_nodes[i].get(), *anchor = dynamic_cast<MyNode*>(balloon->getParent());
		float scale = balloon->getScaleX();
		if(scale > 0.2f) {
			_deflating = true;
			scale *= 0.985f;
			balloon->setScale(scale);
			//adjust it so it is still tangent to the straw
			Vector3 trans = anchor->getTranslationWorld() - straw->getTranslationWorld(), strawAxis;
			straw->getWorldMatrix().transformVector(Vector3::unitZ(), &strawAxis);
			strawAxis.normalize();
			trans -= strawAxis * trans.dot(strawAxis);
			trans = trans.normalize() * (scale * _balloons->_balloonRadius[i] - _balloons->_anchorRadius[i]);
			balloon->setTranslation(trans);	
			//apply the air pressure force
			anchor->getCollisionObject()->asRigidBody()->applyForce(Vector3(0, 0, 40) * scale);
		}
	}
}

void Rocket::launchComplete() {
}

void Rocket::controlEvent(Control *control, Control::Listener::EventType evt) {
	Project::controlEvent(control, evt);
	const char *id = control->getId();
	
	if(strcmp(id, "launch") == 0) {
		_launching = true;
	}
}

bool Rocket::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Project::touchEvent(evt, x, y, contactIndex);
	switch(evt) {
		case Touch::TOUCH_PRESS: {
			break;
		} case Touch::TOUCH_MOVE: {
			break;
		} case Touch::TOUCH_RELEASE: {
			break;
		}
	}
	return true;
}

Rocket::Straw::Straw(Project *project)
  : Project::Element::Element(project, NULL, "straw", "Straw") {
  	_filter = "straw";
}
  
void Rocket::Straw::addPhysics(short n) {

	BoundingBox box = getNode()->getModel()->getMesh()->getBoundingBox();
	Rocket *rocket = (Rocket*)_project;
	rocket->_strawLength = box.max.z - box.min.z;
	rocket->_originalStrawLength = rocket->_strawLength;
	rocket->_strawRadius = (box.max.x - box.min.x) / 2;

	Quaternion rot = Quaternion::identity();
	float angle = atan2(rocket->_strawRadius * 2, rocket->_strawLength);
	Vector3 trans = Vector3::zero(), linearLow(0, 0, -rocket->_pathLength/2), linearHigh(0, 0, rocket->_pathLength/2),
	  angularLow(0, -2*M_PI, -2*M_PI), angularHigh(angle, 2*M_PI, 2*M_PI);
	MyNode *node = getNode();
	node->addPhysics();
	_project->_rootNode->addChild(node);
	_constraint = app->getPhysicsController()->createGenericConstraint(node->getCollisionObject()->asRigidBody(), rot, trans);
	_constraint->setLinearLowerLimit(linearLow);
	_constraint->setLinearUpperLimit(linearHigh);
	_constraint->setAngularLowerLimit(angularLow);
	_constraint->setAngularUpperLimit(angularHigh);
}

bool Rocket::Straw::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Project::Element::touchEvent(evt, x, y, contactIndex);
	switch(evt) {
		case Touch::TOUCH_PRESS: {
			Rocket *rocket = (Rocket*)_project;
			MyNode *node = getNode();
			if(rocket->getTouchNode() == node) {
				Vector3 trans = rocket->getTouchPoint() - node->getTranslationWorld();
				Matrix straw = node->getWorldMatrix(), strawInv;
				straw.invert(&strawInv);
				strawInv.transformVector(&trans);
				float scale = ((rocket->_strawLength/2 - trans.z) / rocket->_strawLength) * node->getScaleZ();
				node->setScaleZ(scale);
				rocket->_strawLength = rocket->_originalStrawLength * scale;
			}
		}
	}
}

Rocket::Balloon::Balloon(Project *project, Element *parent)
  : Project::Element::Element(project, parent, "balloon", "Balloon", true) {
  	_filter = "balloon";
}
  
void Rocket::Balloon::placeNode(short n) {
	Vector3 point = _project->getTouchPoint(_project->getLastTouchEvent());
	//get the unit direction vector 
	Rocket *rocket = (Rocket*)_project;
	MyNode *straw = rocket->_straw->getNode();
	Vector3 trans = point - straw->getTranslationWorld(), strawAxis;
	straw->getWorldMatrix().transformVector(Vector3::unitZ(), &strawAxis);
	strawAxis.normalize();
	trans -= strawAxis * trans.dot(strawAxis);
	trans.normalize();
	//constrain the balloon so it is fixed to the straw
	MyNode *balloon = _nodes.back().get(), *anchor = MyNode::create(MyNode::concat(2, "anchor_", _currentNodeId));
	BoundingBox box = balloon->getModel()->getMesh()->getBoundingBox();
	float balloonRadius = (box.max.x - box.min.x) / 2;
	float anchorRadius = balloonRadius * 0.5f; //best fit to the balloon shape as it deflates?
	if(_balloonRadius.size() <= n) {
		_balloonRadius.resize(n+1);
		_anchorRadius.resize(n+1);
	}
	_balloonRadius[n] = balloonRadius;
	_anchorRadius[n] = anchorRadius;
	anchor->setTranslation(point + trans * anchorRadius);
	anchor->setRotation(straw->getRotation());
	anchor->_objType = "sphere";
	anchor->_radius = anchorRadius;
	anchor->_mass = 0.5f;
	balloon->_objType = "none";
	anchor->addChild(balloon);
	_project->_scene->addNode(anchor);
	balloon->setTranslation(trans * (balloonRadius - anchorRadius));
}

void Rocket::Balloon::addPhysics(short n) {
	MyNode *straw = ((Rocket*)_project)->_straw->getNode(), *balloon = _nodes[n].get(),
	  *anchor = dynamic_cast<MyNode*>(balloon->getParent());
	anchor->addPhysics(false);
	app->addConstraint(straw, anchor, -1, "fixed", Vector3::zero(), Vector3::zero(), true);
}

}



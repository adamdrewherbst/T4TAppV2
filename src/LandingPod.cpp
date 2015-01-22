#include "T4TApp.h"
#include "LandingPod.h"
#include "MyNode.h"

namespace T4T {

LandingPod::LandingPod() : Project::Project("landingPod") {

	app->addItem("podBase", 2, "general", "body");
	app->addItem("hatch1", 2, "general", "hatch");
	app->addItem("hatch2", 2, "general", "hatch");

	_body = (Body*) addElement(new Body(this));
	_hatch = (Hatch*) addElement(new Hatch(this, _body));
	setupMenu();
}

void LandingPod::setupMenu() {
	Project::setupMenu();
	_hatchButton = app->addControl <Button> (NULL, "openHatch");
	_hatchButton->setText("Open Hatch");
	_controls->insertControl(_hatchButton, 2);
	_hatchButton->setEnabled(false);
	app->addListener(_hatchButton, this);
	_controls->setHeight(_controls->getHeight() + 70.0f);	
}

bool LandingPod::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	Project::touchEvent(evt, x, y, contactIndex);
	return true;
}

void LandingPod::controlEvent(Control *control, EventType evt) {
	Project::controlEvent(control, evt);
	const char *id = control->getId();
	
	if(control == _hatchButton) {
		openHatch();
		_hatchButton->setEnabled(false);
	}
}

void LandingPod::setActive(bool active) {
	Project::setActive(active);
}

bool LandingPod::setSubMode(short mode) {
	bool changed = Project::setSubMode(mode);
	switch(_subMode) {
		case 0: { //build
			break;
		} case 1: { //test
			//place the pod at a fixed height
			_rootNode->enablePhysics(false);
			_rootNode->placeRest();
			Vector3 trans(0, 10, 0);
			_rootNode->setMyTranslation(trans);
			//attempt to place the lunar buggy right behind the hatch
			_buggy = app->getProjectNode("buggy");
			if(_buggy && _buggy->getChildCount() > 0) {
				_buggy->enablePhysics(false);
				_buggy->placeRest();
				_buggy->updateTransform();
				_scene->addNode(_buggy);
				MyNode *hatch = _hatch->getNode(), *base = _body->getNode();
				hatch->updateTransform();
				base->updateTransform();
				BoundingBox buggyBox = _buggy->getBoundingBox(true), baseBox = base->getBoundingBox(false, false),
				  hatchBox = hatch->getBoundingBox(false);
				Vector3 normal = -hatch->getJointNormal();
				float backZ = hatch->getMaxValue(normal);
				Vector3 pos = hatch->getTranslationWorld() + (backZ + buggyBox.max.z) * normal;
				pos.y = baseBox.max.y - buggyBox.min.y;
				Quaternion rot = MyNode::getVectorRotation(Vector3::unitZ(), -normal);
				_buggy->setMyTranslation(pos);
				_buggy->setMyRotation(rot);
			}
			//TODO: if not enough space, alert the user
			app->getPhysicsController()->setGravity(Vector3::zero());
			app->_ground->setVisible(true);
			break;
		}
	}
	if(changed) _hatchButton->setEnabled(false);
	_body->_groundAnchor.reset();
	return changed;
}

void LandingPod::launch() {
	Project::launch();
	_rootNode->enablePhysics(true);
	_buggy->enablePhysics(true);
	app->getPhysicsController()->setGravity(app->_gravity);
	_rootNode->setActivation(DISABLE_DEACTIVATION);
	_hatchButton->setEnabled(true);
}

void LandingPod::openHatch() {
	//release the lock and give the hatch an outward kick
	if(_hatch->_lock.get() != nullptr) _hatch->_lock->setEnabled(false);
	//the torque is about the hinge axis
	MyNode *node = _hatch->getNode();
	node->getCollisionObject()->asRigidBody()->applyTorqueImpulse(-node->getJointAxis() * 10.0f);
	//anchor the pod to the ground
	_body->_groundAnchor = ConstraintPtr(app->getPhysicsController()->createFixedConstraint(
	  _body->getNode()->getCollisionObject()->asRigidBody(), app->_ground->getCollisionObject()->asRigidBody()));
	//push the buggy out through the hatch
	if(_buggy->getScene() == _scene) {
		MyNode *body = dynamic_cast<MyNode*>(_buggy->getFirstChild());
		Vector3 impulse = -_buggy->getForwardVector() * 1000.0f;
		body->getCollisionObject()->asRigidBody()->applyImpulse(impulse);
	}
}

LandingPod::Body::Body(Project *project) : Project::Element::Element(project, NULL, "body", "Body") {
	_filter = "body";
}

LandingPod::Hatch::Hatch(Project *project, Element *parent)
  : Project::Element::Element(project, parent, "hatch", "Hatch") {
	_filter = "hatch";
}

void LandingPod::Hatch::placeNode(short n) {
	//put the bottom center of the bounding box where the user clicked
	MyNode *node = _nodes[n].get(), *parent = _parent->getNode();
	node->updateTransform();
	BoundingBox box = node->getBoundingBox(true);
	node->shiftModel(0, -box.min.y, 0);
	node->updateModel(false, false);
	Vector3 point = _project->getTouchPoint(), normal = _project->getTouchNormal(), axis;
	node->attachTo(parent, point, normal);
	//the hinge axis is the tangent to the surface that lies in the xz-plane
	Vector3 normalXZ(normal.x, 0, normal.z);
	if(normalXZ.length() < 1e-4) axis.set(1, 0, 0);
	else axis.set(-normal.z, 0, normal.x);
	node->_parentAxis = axis.normalize();
}

void LandingPod::Hatch::addPhysics(short n) {
	Project::Element::addPhysics(n);
	//the hinge should always be on the bottom edge so the buggy can roll out
	MyNode *node = _nodes[n].get(), *parent = _parent->getNode();
	app->getPhysicsController()->setConstraintNoCollide();
	app->addConstraint(parent, node, -1, "hinge", node->_parentOffset, node->_parentAxis, true);
	//fix the hatch in place until we have landed!
	_lock = ConstraintPtr(app->addConstraint(parent, node, -1, "fixed", node->_parentOffset, node->_parentAxis, false));
}

}

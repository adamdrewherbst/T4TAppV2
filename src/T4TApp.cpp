#include "T4TApp.h"
#include "MyNode.h"
#include "NavigateMode.h"
#include "PositionMode.h"
#include "ConstraintMode.h"
#include "StringMode.h"
#include "TestMode.h"
#include "TouchMode.h"
#include "ToolMode.h"
#include "Satellite.h"
#include "Buggy.h"
#include "Rocket.h"
#include "Robot.h"
#include "LandingPod.h"
#include "Launcher.h"
#include "HullMode.h"
#include "Grid.h"

namespace T4T {

// Declare our game instance
static T4TApp* __t4tInstance = NULL;
static bool _debugFlag = false;
T4TApp game;

T4TApp::T4TApp()
    : _scene(NULL)
{
	__t4tInstance = this;
}

void T4TApp::debugTrigger()
{
	int x = 0;
}

T4TApp* T4TApp::getInstance() {
	return __t4tInstance;
}

void T4TApp::initialize()
{
#ifdef USE_GLU_TESS
	Face::initTess();
#endif
	
	generateModels();
	
    // Load font
    _font = Font::create("res/common/arial18.gpb");
    assert(_font);
    
    _userEmail = "";
    
    initScene();

    cout << "cam at: " << pcam(_cameraState) << endl;
    
	getPhysicsController()->setGravity(Vector3(0.0f, -10.0f, 0.0f));

	_steering = _braking = _driving = 0.0f;
    
    //create the form for selecting catalog items
    cout << "creating theme" << endl;
    _theme = Theme::create("res/common/default.theme");
    cout << "created theme" << endl;
    _formStyle = _theme->getStyle("basicContainer");
    _hiddenStyle = _theme->getStyle("hiddenContainer");
    _buttonStyle = _theme->getStyle("buttonStyle");
    _titleStyle = _theme->getStyle("title");

	/*********************** GUI SETUP ***********************/
	
	//root menu node for finding controls by ID
	_mainMenu = Form::create("res/common/main.form");
	_sideMenu = (Container*)_mainMenu->getControl("sideMenu");
	_stage = (Container*)_mainMenu->getControl("stage");
	_sceneMenu = (Container*)_mainMenu->getControl("submenu_sceneMenu");
	_machineMenu = (Container*)_mainMenu->getControl("submenu_machineMenu");
	_modePanel = (Container*)_sideMenu->getControl("modePanel");
	_cameraMenu = (Container*)_stage->getControl("camera");
	
	ImageControl* itemImage = addControl <ImageControl> (_sideMenu, "dummyImage");
	itemImage->setVisible(false);

	//for selecting items	
	_componentMenu = (Container*)_mainMenu->getControl("submenu_componentMenu");

	//dialogs
	_message = (Label*)_stage->getControl("message");
	_message->setVisible(false);
	_textDialog = (Container*)_mainMenu->getControl("textDialog");
	_textPrompt = (Label*)_textDialog->getControl("textPrompt");
	_textName = (TextBox*)_textDialog->getControl("textName");
	_textSubmit = (Button*)_textDialog->getControl("textSubmit");
	_textCancel = (Button*)_textDialog->getControl("textCancel");
	_textDialog->setVisible(false);
	_confirmDialog = (Container*)_mainMenu->getControl("confirmDialog");
	_confirmMessage = (Label*)_confirmDialog->getControl("confirmMessage");
	_confirmYes = (Button*)_confirmDialog->getControl("confirmYes");
	_confirmNo = (Button*)_confirmDialog->getControl("confirmNo");
	_confirmDialog->setVisible(false);
	_overlay = (Container*)_mainMenu->getControl("overlay");
	_overlay->setVisible(false);
	
	_undo = (Button*)_mainMenu->getControl("undo");
	_redo = (Button*)_mainMenu->getControl("redo");
	_undo->setEnabled(false);
	_redo->setEnabled(false);
    
	//identify all submenus so we can close any open ones when another is clicked
	std::vector<Control*> controls = _mainMenu->getControls();
	Container *container;
	for(std::vector<Control*>::iterator it = controls.begin(); it != controls.end(); it++) {
		container = dynamic_cast<Container*>(*it);
		if(container && strncmp(container->getId(), "submenu_", 8) == 0) {
			_submenus.push_back(container);
			container->setVisible(false);
		}
	}
	
	// populate catalog of items
	_models = Scene::create("models");
	addItem("box", 2, "general", "body");
	addItem("sphere", 2, "general", "body");
	addItem("cylinder", 2, "general", "body");
	addItem("halfpipe", 3, "general", "body", "lever arm");
	addItem("gear_basic", 2, "general", "gear");
	addItem("cap_with_hole_1", 1, "general");
	addItem("pitri_wheel", 1, "general");
	addItem("green_flange_wheel", 1, "general");
	addItem("jar_with_cone", 1, "general");

	_drawDebugCheckbox = (CheckBox*) _sideMenu->getControl("drawDebug");
	//_drawDebugCheckbox = addControl <CheckBox> (_sideMenu, "drawDebug");
	//Button *debugButton = addControl <Button> (_sideMenu, "debugButton");
	
    addListener(_mainMenu, this);
    //addListener(_textName, this, Control::Listener::TEXT_CHANGED);
	
	//interactive modes
	_modes.push_back(new NavigateMode());
	_modes.push_back(new PositionMode());
	_modes.push_back(new ConstraintMode());
	_modes.push_back(new Satellite());
	_modes.push_back(new Rocket());
	_modes.push_back(new Buggy());
	_modes.push_back(new Robot());
	_modes.push_back(new LandingPod());
	_modes.push_back(new Launcher());
	_modes.push_back(new HullMode());
	_modes.push_back(new StringMode());
	_modes.push_back(new ToolMode());
	_modes.push_back(new TestMode());
	_modes.push_back(new TouchMode());

	//simple machines
    //_modes.push_back(new Lever());
    //_modes.push_back(new Pulley());
    
    _itemFilter = new MenuFilter(_componentMenu);
    
	//for queuing user actions for undo/redo
    _action = NULL;
    _tmpNode = MyNode::create("tmpNode");
    _tmpCount = 0;

    //exclude certain nodes (eg. ground, camera) from being selected by touches
    _hitFilter = new HitFilter(this);
    _nodeFilter = new NodeFilter();
    
	//nodes to illustrate mesh pieces when debugging    
	_face = MyNode::create("face");
	_face->_wireframe = true;
	_face->_lineWidth = 5.0f;
	_edge = MyNode::create("edge");
	_edge->_wireframe = true;
	_edge->_lineWidth = 5.0f;
	_vertex = duplicateModelNode("sphere");
	_vertex->setScale(0.15f);
	_vertex->getModel()->setMaterial("res/common/models.material#red");

	_activeMode = -1;
	setMode(0);

	_drawDebug = true;	
    _sideMenu->setFocus();
    
	_running = 0;
	_constraintCount = 0;
	
	resizeEvent(getWidth(), getHeight());
}

void T4TApp::resizeEvent(unsigned int width, unsigned int height) {

	_stage->setWidth(width - _sideMenu->getWidth());
	_stage->setHeight(height);

	_componentMenu->setPosition(_sideMenu->getX() + _sideMenu->getWidth() + 25.0f, 25.0f);
	_componentMenu->setWidth(width - 2 * _componentMenu->getX());
	_componentMenu->setHeight(height - 2 * _componentMenu->getY());

	_message->setPosition(0.0f, height - _message->getHeight());
}

void T4TApp::finalize()
{
    SAFE_RELEASE(_scene);
    SAFE_RELEASE(_mainMenu);
}

int updateCount = 0;
void T4TApp::update(float elapsedTime)
{
	if(_activeMode >= 0) _modes[_activeMode]->update();
    _mainMenu->update(elapsedTime);
	if(_carVehicle) _carVehicle->update(elapsedTime, _steering, _braking, _driving);
}

void T4TApp::render(float elapsedTime)
{
    // Clear the color and depth buffers
    clear(CLEAR_COLOR_DEPTH, Vector4::zero(), 1.0f, 0);

    // Visit all the nodes in the scene for drawing
    if(_activeScene != NULL) {
    	_activeScene->visit(this, &T4TApp::drawNode);
    	if(_drawDebug) getPhysicsController()->drawDebug(_activeScene->getActiveCamera()->getViewProjectionMatrix());
    }

    // Draw text
    Vector4 fontColor(1.0f, 1.0f, 1.0f, 1.0f);
    unsigned int width, height;
    char buffer[50];

/*    _font->start();

    // Mouse
    sprintf(buffer, "M(%d,%d)", (int)_mousePoint.x, (int)_mousePoint.y);
    _font->measureText(buffer, _font->getSize(), &width, &height);
    int x = _mousePoint.x - (int)(width>>1);
    int y = _mousePoint.y - (int)(height>>1);
    //cout << "drawing " << buffer << " at " << x << ", " << y << endl;
    _font->drawText(buffer, x, y, fontColor, _font->getSize());
    if (_mouseString.length() > 0)
    {
        int y = getHeight() - _font->getSize();
        _font->drawText(_mouseString.c_str(), 0, y, fontColor, _font->getSize());
    }
    _font->finish();
//*/
	_mainMenu->draw();
}

void T4TApp::login() {
	
}

void T4TApp::redraw() {
	render(0);
	Platform::swapBuffers();
}

void T4TApp::setMode(short mode) {
	mode %= _modes.size();
	if(_activeMode == mode) return;
	if(_activeMode >= 0) _modes[_activeMode]->setActive(false);
	_activeMode = mode;
	if(_activeMode >= 0) _modes[_activeMode]->setActive(true);
}

void T4TApp::setNavMode(short mode) {
	_navMode = mode;
}

void T4TApp::controlEvent(Control* control, Control::Listener::EventType evt)
{
	const char *id = control->getId();
	Container *parent = (Container*) control->getParent();
	cout << "CLICKED " << id << endl;

	//scene operations
	if(_sceneMenu->getControl(id) == control) {
		if(strcmp(id, "new") == 0) {
			clearScene();
			setSceneName("test");
		}
		else if(strcmp(id, "load") == 0) {
			getText("Enter scene name: ", "Load", &T4TApp::loadScene);
		}
		else if(strcmp(id, "save") == 0) {
			saveScene();
		}
		else if(strcmp(id, "saveAs") == 0) {
			getText("Enter scene name:", "Save", &T4TApp::saveScene);
		}
	}	
	//callbacks for modal dialogs
	else if((control == _textName && evt == TEXT_CHANGED &&
	  (_textName->getLastKeypress() == 10 || _textName->getLastKeypress() == 13))
	  || control == _textSubmit) {
		if(_textCallback != NULL) (this->*_textCallback)(_textName->getText());
		showDialog(_textDialog, false);
		//when the enter key is released, if there is an active button in the focused container, it will fire
		inactivateControls();
	}
	else if(control == _textCancel) {
		_textCallback = NULL;
		showDialog(_textDialog, false);
	}
	else if(control == _confirmYes || control == _confirmNo) {
		if(_confirmCallback) (this->*_confirmCallback)(control == _confirmYes);
		showDialog(_confirmDialog, false);
	}

	//if a submenu handle is clicked, toggle whether the submenu is expanded
	else if(strncmp(id, "parent_", 7) == 0) {
		const char *subName = MyNode::concat(2, "submenu_", id+7);
		cout << "Looking for submenu " << subName << endl;
		Container *subMenu = dynamic_cast<Container*>(_mainMenu->getControl(subName));
		if(subMenu) {
			bool visible = subMenu->isVisible();
			for(size_t i = 0; i < _submenus.size(); i++)
				_submenus[i]->setVisible(false);
			cout << "\ttoggling menu to " << !visible << endl;
			subMenu->setVisible(!visible);
			if(!visible) { //if expanding the submenu, position it next to its handle
				if(subMenu != _componentMenu) {
					float x = control->getX() + control->getWidth(), y = control->getY();
					subMenu->setPosition(x, y);
					cout << "\tpositioned at " << x << ", " << y << endl;
				}
			}
		} else {
			cout << "No control with ID " << subName << endl;
		}
	}

	//misc submenu funcionality
	else if(_cameraMenu->getControl(id) == control) {
		if(strcmp(id, "eye") == 0) {
			if(_navMode >= 0) setNavMode(-1);
			else setNavMode(0);
		} else if(strcmp(id, "rotate") == 0) {
			setNavMode(0);
		} else if(strcmp(id, "translate") == 0) {
			setNavMode(1);
		} else if(strcmp(id, "zoom") == 0) {
			setNavMode(2);
		} else if(strcmp(id, "reset") == 0) {
			resetCamera();
		}
	}
	else if(_modePanel->getControl(id) == control || _machineMenu->getControl(id) == control) {
		//simple machines and interactive modes are functionally equivalent, just in different submenus
		for(short i = 0; i < _modes.size(); i++) {
			if(strcmp(_modes[i]->getId(), id) == 0) setMode(i);
		}
	}
	else if(_componentMenu->getControl(id) == control && strncmp(id, "comp_", 5) == 0) {
		bool consumed = false;
		if(_activeMode >= 0) consumed = _modes[_activeMode]->selectItem(id+5);
		if(!consumed) {
			MyNode *node = addModelNode(id+5);
			setAction("addNode", node);
			commitAction();
		}
	}
	else if(strcmp(id, "debugButton") == 0) {
		debugTrigger();
	}
	else if(control == _drawDebugCheckbox) {
		_drawDebug = _drawDebugCheckbox->isChecked();
	}
	
	//undo/redo
	else if(control == _undo) {
		undoLastAction();
	}
	else if(control == _redo) {
		redoLastAction();
	}

	//close a submenu when one of its items is clicked
	Container *next = parent;
	while(next != NULL && strncmp(next->getId(), "submenu_", 8) == 0) {
		next->setVisible(false);
		Control *handle = _mainMenu->getControl(MyNode::concat(2, "parent_", next->getId()));
		//if(handle != NULL) handle->setState(Control::NORMAL);
		next = (Container*) next->getParent();
	}
}

void T4TApp::keyEvent(Keyboard::KeyEvent evt, int key) {
	if(_activeMode >= 0 && _modes.size() > _activeMode) _modes[_activeMode]->keyEvent(evt, key);
}
void T4TApp::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex) {
	//cout << "Touched! " << x << "," << y << endl;
	if(_activeMode >= 0 && _modes.size() > _activeMode) _modes[_activeMode]->touchEvent(evt, x, y, contactIndex);
}
bool T4TApp::mouseEvent(Mouse::MouseEvent evt, int x, int y, int wheelDelta) {
	//cout << "Moused! " << x << "," << y << endl;
	return false;
}

void T4TApp::inactivateControls(Container *container) {
	if(container == NULL) container = _mainMenu;
	std::vector<Control*> controls = container->getControls();
	for(std::vector<Control*>::iterator it = controls.begin(); it != controls.end(); it++) {
		Control *control = *it;
		//if(control->getState() == Control::ACTIVE) control->setState(Control::NORMAL);
		if(control->isContainer()) inactivateControls((Container*)control);
	}
}

void T4TApp::initScene()
{
    // Generate game scene
    _scene = Scene::load("res/common/game.scene");
    _scene->setId("scene");
    setSceneName("test");
    _scene->visit(this, &T4TApp::printNode);

    // Set the aspect ratio for the scene's camera to match the current resolution
    _scene->getActiveCamera()->setAspectRatio(getAspectRatio());
    _cameraState = new cameraState();

    // Get light node
    _lightNode = _scene->findNode("lightNode");
    _light = _lightNode->getLight();

	//create the grid on which to place objects
	short gridSize = 40;
    _ground = MyNode::create("grid");
    Model* gridModel = createGridModel(2 * gridSize + 1);
    gridModel->setMaterial("res/common/models.material#grid");
    _ground->setModel(gridModel);
    gridModel->release();
    _ground->_objType = "box";
    _ground->setStatic(true);
    //store the plane representing the grid, for calculating intersections
    _groundPlane = Plane(Vector3(0, 1, 0), 0);
    
    //add invisible walls at the edges of the grid to prevent objects falling off the world
    for(short i = 0; i < 2; i++) {
    	for(short j = 0; j < 2; j++) {
    		std::ostringstream os;
			os << "wall" << i*2+j;
			MyNode *wall = MyNode::create(os.str().c_str());
			wall->_objType = "box";
			Vector3 extents(0, 5, 0), center(0, 4, 0);
			if(i == 0) {
				center.x = (2*j-1) * (gridSize+1);
				extents.x = 1;
				extents.z = gridSize;
			} else {
				center.z = (2*j-1) * (gridSize+1);
				extents.x = gridSize;
				extents.z = 1;
			}
			wall->_boundingBox.set(center - extents, center + extents);
			_ground->addChild(wall);
		}
    }

    _ground->addPhysics();

    //create lines for the positive axes
    std::vector<float> vertices;
    vertices.resize(36,0);
    for(int i = 0; i < 6; i++) {
    	if(i%2 == 1) vertices[i*6+i/2] += 5.0f;
    	vertices[i*6+3+i/2] = 1.0f; //axes are R, G, and B
    }
    _axes = createWireframe(vertices, "axes");

    _gravity.set(0, -10, 0);
    getPhysicsController()->setGravity(_gravity);

    setActiveScene(_scene);
    resetCamera();
}

void T4TApp::addItem(const char *type, short numTags, ...) {
	if(_models->findNode(type) != NULL) return;
	va_list args;
	va_start(args, numTags);
	MyNode *node = MyNode::create(type);
	node->_type = type;
	node->loadData("res/common/", false);
	node->setTranslation(Vector3(1000.0f,0.0f,0.0f));
	for(short i = 0; i < numTags; i++) {
		const char *tag = va_arg(args, const char*);
		node->setTag(tag);
	}
	va_end(args);
	_models->addNode(node);
	//ImageControl* itemImage = addControl <ImageControl> (_componentMenu, MyNode::concat(2, "comp_", type));
	Button *itemImage = addControl <Button> (_componentMenu, MyNode::concat(2, "comp_", type), NULL, 150.0f, 150.0f);
	itemImage->setText(type);
	itemImage->setZIndex(_componentMenu->getZIndex());
	//itemImage->setImage("res/png/cowboys-helmet-nobkg.png");
}

void T4TApp::filterItemMenu(const char *tag) {
	for(Node *n = _models->getFirstNode(); n; n = n->getNextSibling()) {
		MyNode *node = dynamic_cast<MyNode*>(n);
		bool filtered = tag && !node->hasTag(tag);
		const char *id = MyNode::concat(2, "comp_", node->getId());
		_itemFilter->filter(id, filtered);
	}
}

void T4TApp::promptItem(const char *tag) {
	filterItemMenu(tag);
	_componentMenu->setVisible(true);
}

void T4TApp::setSceneName(const char *name) {
	_sceneName = name;
}

std::string T4TApp::getSceneDir() {
	return "res/scenes/" + _sceneName + "_";
}

void T4TApp::loadScene(const char *scene) {
	std::string oldName = _sceneName;
	if(scene != NULL) setSceneName(scene);
	std::string listFile = getSceneDir() + "scene.list", id;
	std::unique_ptr<Stream> stream(FileSystem::open(listFile.c_str()));
	if(stream.get() == NULL) {
		cout << "Failed to open file" << listFile << endl;
		setSceneName(oldName.c_str());
		return;
	}
	clearScene();
	char line[256], *str;
	std::istringstream ss;
	while(!stream->eof()) {
		str = stream->readLine(line, 256);
		ss.str(str);
		ss >> id;
		loadNode(id.c_str());
	}
}

MyNode* T4TApp::loadNode(const char *id) {
	MyNode *node = MyNode::create(id);
	_scene->addNode(node);
	node->loadData();
	node->enablePhysics();
	return node;
}

void T4TApp::saveScene(const char *scene) {
	if(scene != NULL) setSceneName(scene);
	//create a file that lists all root nodes in the scene, and save each one to its own file
	std::string listFile = getSceneDir() + "scene.list", line;
	std::unique_ptr<Stream> stream(FileSystem::open(listFile.c_str(), FileSystem::WRITE));
	if(stream.get() == NULL) {
		GP_ERROR("Failed to open file '%s'", listFile.c_str());
		return;
	}
	MyNode *node;
	std::ostringstream os;
	for(Node *n = _scene->getFirstNode(); n != NULL; n = n->getNextSibling()) {
		if(n->getParent() != NULL || auxNode(n)) continue;
		node = dynamic_cast<MyNode*>(n);
		if(node) {
			os.str("");
			os << node->getId() << endl;
			line = os.str();
			stream->write(line.c_str(), sizeof(char), line.length());
			node->writeData();
		}
	}
	stream->close();
}

bool T4TApp::saveNode(Node *n) {
	MyNode *node = dynamic_cast<MyNode*>(n);
	if(node && !node->getParent() && !auxNode(node)) node->writeData();
	return true;
}

void T4TApp::clearScene() {
	std::vector<MyNode*> nodes;
	for(Node *n = _scene->getFirstNode(); n != NULL; n = n->getNextSibling()) {
		if(n->getParent() != NULL || auxNode(n)) continue;
		MyNode *node = dynamic_cast<MyNode*>(n);
		if(node) nodes.push_back(node);
	}
	for(short i = 0; i < nodes.size(); i++) {
		nodes[i]->removePhysics();
		_scene->removeNode(nodes[i]);
	}
}

void T4TApp::removeNode(MyNode *node, bool erase) {
	if(erase) {
		node->removePhysics();
	} else {
		node->addRef();
		node->enablePhysics(false);
	}
	_scene->removeNode(node);
}

bool T4TApp::auxNode(Node *node) {
	const char *id = node->getId();
	return strcmp(id, "grid") == 0 || strcmp(id, "axes") == 0 || strcmp(id, "camera") == 0;
}

void T4TApp::releaseScene()
{
	if(_activeScene == _scene) _activeScene = NULL;
	_carVehicle = NULL;
	SAFE_RELEASE(_scene);
}

void T4TApp::showScene() {
	setActiveScene(_scene);
}

bool T4TApp::hideNode(Node *node) {
	MyNode *n = dynamic_cast<MyNode*>(node);
	if(n) n->enablePhysics(false);
	return true;
}

bool T4TApp::showNode(Node *node) {
	MyNode *n = dynamic_cast<MyNode*>(node);
	if(n) n->enablePhysics(true);
	return true;
}

Project* T4TApp::getProject(const char *id) {
	short i, n = _modes.size();
	for(i = 0; i < n; i++) {
		Project *project = dynamic_cast<Project*>(_modes[i]);
		if(project && strcmp(project->getId(), id) == 0) return project;
	}
	return NULL;
}

MyNode* T4TApp::getProjectNode(const char *id) {
	Project *project = getProject(id);
	if(project) return project->_rootNode;
	return NULL;
}

Form* T4TApp::addMenu(const char *name, Container *parent, const char *buttonText, Layout::Type layout)
{
	bool isSubmenu = parent != NULL && buttonText != NULL;
	const char *id = name;
	if(isSubmenu) id = MyNode::concat(2, "submenu_", name);
    Form *container = Form::create(id, _formStyle, layout);
    if(parent == NULL || parent == _mainMenu) container->setAutoSize(Control::AUTO_SIZE_HEIGHT);
	else container->setHeight(300.0f);
    container->setWidth(200.0f);
    container->setScroll(Container::SCROLL_VERTICAL);
    container->setConsumeInputEvents(true);
    if(isSubmenu) {
    	container->setZIndex(parent->getZIndex() + 10);
		container->setVisible(false);
		_mainMenu->addControl(container);
		_submenus.push_back(container);
    	Button *toggle = addControl<Button>(parent, MyNode::concat(2, "parent_", name));
    	toggle->setText(buttonText);
    }
    else if(parent != NULL) parent->addControl(container);
    return container;
}

Form* T4TApp::addPanel(const char *name, Container *parent)
{
	Form *container = Form::create(name, _formStyle, Layout::LAYOUT_VERTICAL);
	container->setHeight(250);
	container->setWidth(175);
	container->setScroll(Container::SCROLL_NONE);
    container->setConsumeInputEvents(true);
    if(parent != NULL) parent->addControl(container);
    return container;
}

void T4TApp::addListener(Control *control, Control::Listener *listener, int evtFlags) {
	enableListener(true, control, listener, evtFlags);
}

void T4TApp::removeListener(Control *control, Control::Listener *listener) {
	enableListener(false, control, listener);
}

void T4TApp::enableListener(bool enable, Control *control, Control::Listener *listener, int evtFlags) {
	if(enable) control->addListener(listener, evtFlags);
	else control->removeListener(listener);
	Container *container = dynamic_cast<Container*>(control), *submenu;
	if(container) {
		std::vector<Control*> controls = container->getControls();
		for(int i = 0; i < controls.size(); i++) {
			const char *id = controls[i]->getId(), *submenuID;
			if(strncmp(id, "submenu_", 8) != 0) {
				enableListener(enable, controls[i], listener, evtFlags);
			}
			if(strncmp(id, "parent_", 7) == 0) {
				submenuID = MyNode::concat(2, "submenu_", id+7);
				submenu = dynamic_cast<Container*>(_mainMenu->getControl(submenuID));
				if(submenu) enableListener(enable, submenu, listener, evtFlags);
			}
		}
	}
}

void T4TApp::getText(const char *prompt, const char *type, void (T4TApp::*callback)(const char*)) {
	_textCallback = callback;
	_textPrompt->setText(prompt);
	_textSubmit->setText(type);
	_textName->setText("");
	showDialog(_textDialog);
	//_textName->setState(Control::ACTIVE);
}

void T4TApp::doConfirm(const char *message, void (T4TApp::*callback)(bool)) {
	_confirmCallback = callback;
	_confirmMessage->setText(message);
	showDialog(_confirmDialog);
}

void T4TApp::showDialog(Container *dialog, bool show) {
	_overlay->setVisible(show);
	dialog->setVisible(show);
	//if(show) dialog->setState(Control::FOCUS);
}

void T4TApp::confirmDelete(bool yes) {
	if(!yes) return;
	if(_activeMode < 0) return;
	MyNode *node = _modes[_activeMode]->_selectedNode;
	if(node != NULL) {
		setAction("deleteNode", node);
		removeNode(node, false);
		commitAction();
	}
}

void T4TApp::message(const char *text) {
	if(text == NULL) {
		_message->setVisible(false);
	} else {
		_message->setText(text);
		_message->setVisible(true);
	}
}

bool T4TApp::hasMessage() {
	return _message->isVisible();
}

bool T4TApp::prepareNode(MyNode* node)
{
	PhysicsCollisionObject* obj = node->getCollisionObject();
	if(obj && obj->asRigidBody()) {
		cout << "adding collision listener to " << node->getId() << endl;
		obj->asRigidBody()->addCollisionListener(this);
	}
}

bool T4TApp::printNode(Node *node) {
	Node *parent = node->getParent();
	if(parent != NULL) {
		cout << parent->getId() << ";" << parent->getType() << " => ";
	}
	cout << node->getId() << ";" << node->getType() << endl;
	return true;
}

bool T4TApp::drawNode(Node* node)
{
    Model* model = node->getModel();
    if (model) {
		bool wireframe = false;
		float lineWidth = 1.0f;
    	MyNode *myNode = dynamic_cast<MyNode*>(node);
    	if(myNode) {
    		if(!myNode->_visible) return true;
    		wireframe = myNode->_wireframe || myNode->_chain;
    		lineWidth = myNode->_lineWidth;
    	}
    	if(wireframe) glLineWidth(lineWidth);
    	model->draw(wireframe);
    	if(wireframe) glLineWidth(1.0f);
    }
    return true;
}

MyNode* T4TApp::getMouseNode(int x, int y, Vector3 *touch) {
	Camera* camera = _scene->getActiveCamera();
	Ray ray;
	camera->pickRay(getViewport(), x, y, &ray);
	PhysicsController::HitResult hitResult;
	if(!getPhysicsController()->rayTest(ray, camera->getFarPlane(), &hitResult)) return NULL;
	if(touch) touch->set(hitResult.point);
	MyNode *node = dynamic_cast<MyNode*>(hitResult.object->getNode());
	if(!node || strcmp(node->getId(), "grid") == 0) return NULL;
	return node;
}

//see if the current touch location intersects the bottom face of the given object
bool T4TApp::checkTouchModel(Node* n)
{
	MyNode *node = dynamic_cast<MyNode*>(n);
	if(!node) return true;
	if(strcmp(node->getScene()->getId(), "models") == 0) return true;
	if(strcmp(node->getId(), "knife") == 0) return true;
	if(strcmp(node->getId(), "drill") == 0) return true;
	for(int i = 0; i < _intersectNodeGroup.size(); i++)
		if(node == _intersectNodeGroup[i]) return true;
	Vector3 pos = node->getTranslation();
	BoundingBox bbox = node->getBoundingBox();
	if(bbox.isEmpty()) return true;
	float halfX = (_intersectBox.max.x - _intersectBox.min.x) / 2.0f,
		halfY = (_intersectBox.max.y - _intersectBox.min.y) / 2.0f,
		halfZ = (_intersectBox.max.z - _intersectBox.min.z) / 2.0f;
	if(_intersectPoint.x + halfX > pos.x + bbox.min.x && _intersectPoint.x - halfX < pos.x + bbox.max.x
		&& _intersectPoint.z + halfZ > pos.z + bbox.min.z && _intersectPoint.z - halfZ < pos.z + bbox.max.z)
	{
		if(_intersectModel == NULL || halfY + pos.y + bbox.max.y > _intersectPoint.y)
		{
			_intersectModel = node;
			_intersectPoint.y = pos.y + bbox.max.y + halfY;
		}
	}
	return true;
}

MyNode* T4TApp::duplicateModelNode(const char* type, bool isStatic)
{
	MyNode *modelNode = dynamic_cast<MyNode*>(_models->findNode(type));
	if(!modelNode) return NULL;
	MyNode *node = MyNode::cloneNode(modelNode);
	BoundingBox box = node->getModel()->getMesh()->getBoundingBox();
	std::ostringstream os;
	os << modelNode->getId() << ++modelNode->_typeCount;
	node->setId(os.str().c_str());
	node->setTranslation(Vector3(0.0f, (box.max.y - box.min.y)/2.0f, 0.0f));
	node->setStatic(isStatic);
	node->updateAll();
	return node;
}

MyNode* T4TApp::addModelNode(const char *type) {
	MyNode *node = duplicateModelNode(type);
	node->addPhysics();
	_scene->addNode(node);
	placeNode(node, 0.0f, 0.0f);
	node->updateMaterial();
	return node;
}

Model* T4TApp::createModel(std::vector<float> &vertices, bool wireframe, const char *material, Node *node) {
	int numVertices = vertices.size()/6;
	VertexFormat::Element elements[] = {
		VertexFormat::Element(VertexFormat::POSITION, 3),
		VertexFormat::Element(wireframe ? VertexFormat::COLOR : VertexFormat::NORMAL, 3)
	};
	Mesh* mesh = Mesh::createMesh(VertexFormat(elements, 2), numVertices, false);
	mesh->setPrimitiveType(wireframe ? Mesh::LINES : Mesh::TRIANGLES);
	mesh->setVertexData(&vertices[0], 0, numVertices);
	Model *model = Model::create(mesh);
	mesh->release();
	model->setMaterial(MyNode::concat(2, "res/common/models.material#", material));
	if(node) {
		if(node->getModel()) node->setModel(NULL);
		node->setModel(model);
		model->release();
	}
	return model;
}

MyNode* T4TApp::createWireframe(std::vector<float>& vertices, const char *id) {
	if(id == NULL) id = "wireframe1";
	MyNode *node = MyNode::create(id);
	createModel(vertices, true, "grid", node);
	return node;
}

MyNode* T4TApp::dropBall(Vector3 point) {
	MyNode *node = duplicateModelNode("sphere");
	node->setScale(0.3f);
	node->setTranslation(point);
	node->_mass = 3.0f;
	node->addCollisionObject();
	_scene->addNode(node);
	node->updateMaterial();
	cout << "added ball at " << pv(node->getTranslationWorld()) << endl;
	return node;
}

//place at the given xz-coords and set its y-coord so it is on top of any objects it would otherwise intersect
void T4TApp::placeNode(MyNode *node, float x, float z)
{
	_intersectNodeGroup.clear();
	_intersectNodeGroup.push_back(node);
	_intersectBox = node->getBoundingBox();
	float minY = _intersectBox.min.y;
	node->setTranslation(x, -minY, z); //put the bounding box bottom on the ground
	_intersectPoint.set(x, -minY, z);
	_intersectModel = NULL;
	_scene->visit(this, &T4TApp::checkTouchModel); //will change _intersectPoint.y to be above any intersecting models
	node->setTranslation(_intersectPoint);
}

void T4TApp::showFace(Meshy *mesh, std::vector<unsigned short> &face, bool world) {
	_debugMesh = mesh;
	_debugFace = face;
	_debugWorld = world;
	Vector3 vec;
	short n = face.size(), i;
	std::vector<Vector3> vFace;
	for(i = 0; i < n; i++) {
		vec = mesh->_vertices[face[i]];
		if(world) mesh->_node->getWorldMatrix().transformPoint(&vec);
		vFace.push_back(vec);
	}
	showFace(mesh, vFace);
}

void T4TApp::showFace(Meshy *mesh, std::vector<Vector3> &face) {
	short i, j, k, n = face.size(), size = 6 * 2 * n, v = size-6;
	std::vector<float> vertices(size);
	Vector3 norm = Meshy::getNormal(face), vec;
	float color[3] = {1.0f, 0.0f, 1.0f};
	for(i = 0; i < n; i++) {
		vec = face[i] + 0.05f * norm;
		for(j = 0; j < 2; j++) {
			for(k = 0; k < 3; k++) vertices[v++] = MyNode::gv(vec, k);
			for(k = 0; k < 3; k++) vertices[v++] = color[k];
			v %= size;
		}
	}
	createModel(vertices, true, "grid", _face);
	_activeScene->addNode(_face);
	MyNode *node = dynamic_cast<MyNode*>(mesh->_node);
	if(node) node->setWireframe(true);
	frame();
	Platform::swapBuffers();
}

void T4TApp::showEdge(short e) {
	short i, j, n = _debugFace.size(), v = 0;
	if(_debugMesh == NULL || e >= n) return;
	Vector3 vec;
	std::vector<float> vertices(12);
	float color[3] = {1.0f, 1.0f, 1.0f};
	for(i = 0; i < 2; i++) {
		vec = _debugMesh->_vertices[_debugFace[(e+i)%n]];
		if(_debugWorld) _debugMesh->_node->getWorldMatrix().transformPoint(&vec);
		for(j = 0; j < 3; j++) vertices[v++] = MyNode::gv(vec, j);
		for(j = 0; j < 3; j++) vertices[v++] = color[j];
	}
	createModel(vertices, true, "grid", _edge);
	_activeScene->addNode(_edge);
	frame();
	Platform::swapBuffers();
}

void T4TApp::showVertex(short v) {
	if(_debugMesh == NULL || v >= _debugMesh->nv()) return;
	Vector3 vec = _debugMesh->_vertices[v];
	if(_debugWorld) _debugMesh->_node->getWorldMatrix().transformPoint(&vec);
	_vertex->setTranslation(vec);
	_activeScene->addNode(_vertex);
	frame();
	Platform::swapBuffers();
}

void T4TApp::showEye(float radius, float theta, float phi) {
	setCameraEye(radius, theta, phi);
	frame();
	Platform::swapBuffers();
}

void T4TApp::setActiveScene(Scene *scene)
{
	if(scene == _activeScene) return;
	if(_activeScene != NULL) {
		_activeScene->removeNode(_face);
		_activeScene->visit(this, &T4TApp::hideNode);
	}
	_activeScene = scene;
	if(_activeScene != NULL) {
		_activeScene->addNode(_axes);
		_activeScene->addNode(_ground);
		_activeScene->visit(this, &T4TApp::showNode);
	}
}

Camera* T4TApp::getCamera() {
	return _activeScene->getActiveCamera();
}

Node* T4TApp::getCameraNode() {
	return getCamera()->getNode();
}

void T4TApp::placeCamera() {
	float radius = _cameraState->radius, theta = _cameraState->theta, phi = _cameraState->phi;
	Vector3 eye(radius * cos(theta) * cos(phi), radius * sin(phi), radius * sin(theta) * cos(phi));
	eye += _cameraState->target;
	Vector3 up(-cos(theta) * sin(phi), cos(phi), -sin(theta) * sin(phi));
	Matrix cam;
	Matrix::createLookAt(eye, _cameraState->target, up, &cam);
	cam.invert();
	if(_cameraState->node != NULL) {
		Matrix node = _cameraState->node->getRotTrans(), camCopy = cam;
		Matrix::multiply(node, camCopy, &cam);
	}
	Vector3 scale, translation; Quaternion rotation;
	cam.decompose(&scale, &rotation, &translation);
	getCameraNode()->set(scale, rotation, translation);
}

void T4TApp::setCameraEye(float radius, float theta, float phi) {
	_cameraState->radius = radius;
	_cameraState->theta = theta;
	_cameraState->phi = phi;
	placeCamera();
}

void T4TApp::setCameraZoom(float radius) {
	_cameraState->radius = radius;
	placeCamera();
}

void T4TApp::setCameraTarget(Vector3 target) {
	_cameraState->target = target;
	placeCamera();
}

void T4TApp::setCameraNode(MyNode *node) {
	if(node != NULL) {
		cameraPush();
		_cameraState->node = node;
		resetCamera();
		if(_navMode == 1) setNavMode(0);
	} else {
		cameraPop();
	}
	_cameraMenu->getControl("translate")->setEnabled(node == NULL);
}

void T4TApp::resetCamera() {
	_cameraState->target.set(0, 0, 0);
	if(_cameraState->node == NULL) {
		_cameraState->radius = 30;
		_cameraState->theta = M_PI / 2;
		_cameraState->phi = M_PI / 12;
	} else {
		_cameraState->radius = 20;
		_cameraState->theta = -M_PI/2;
		_cameraState->phi = 0;
	}
	placeCamera();
}

cameraState* T4TApp::copyCameraState(cameraState *state, cameraState *dst) {
	if(dst == NULL) dst = new cameraState();
	dst->node = state->node;
	dst->radius = state->radius;
	dst->theta = state->theta;
	dst->phi = state->phi;
	dst->target = state->target;
	return dst;
}

void T4TApp::cameraPush() {
	cameraState *state = copyCameraState(_cameraState);
	_cameraHistory.push_back(state);
}

void T4TApp::cameraPop() {
	cameraState *state = _cameraHistory.back();
	copyCameraState(state, _cameraState);
	_cameraHistory.pop_back();
	placeCamera();
}

const std::string T4TApp::pv(const Vector3& v) {
	std::ostringstream os;
	os << "<" << v.x << "," << v.y << "," << v.z << ">";
	return os.str();
}

const std::string T4TApp::pv2(const Vector2& v) {
	std::ostringstream os;
	os << "<" << v.x << "," << v.y << ">";
	return os.str();
}

const std::string T4TApp::pq(const Quaternion& q) {
	std::ostringstream os;
	Vector3 axis;
	float ang = q.toAxisAngle(&axis);
	os << "<" << axis.x << "," << axis.y << "," << axis.z << " ; " << (int)(ang*180/M_PI) << ">";
	return os.str();
}

const std::string T4TApp::pcam(cameraState *state) {
	std::ostringstream os;
	os << state->radius << "," << state->theta << "," << state->phi << " => " << pv(state->target);
	if(state->node != NULL) os << " [node " << state->node->getId() << "]";
	return os.str();
}

void T4TApp::collisionEvent(PhysicsCollisionObject::CollisionListener::EventType type, 
                            const PhysicsCollisionObject::CollisionPair& pair, 
                            const Vector3& pointA, const Vector3& pointB)
{
    GP_WARN("Collision between rigid bodies %s (at point (%f, %f, %f)) "
            "and %s (at point (%f, %f, %f)).",
            pair.objectA->getNode()->getId(), pointA.x, pointA.y, pointA.z, 
            pair.objectB->getNode()->getId(), pointB.x, pointB.y, pointB.z);
}

/********** PHYSICS ***********/

PhysicsConstraint* T4TApp::addConstraint(MyNode *n1, MyNode *n2, int id, const char *type,
  const Vector3 &joint, const Vector3 &direction, bool parentChild) {
	Quaternion rotOffset = MyNode::getVectorRotation(Vector3::unitZ(), direction),
	  rot1 = PhysicsConstraint::getRotationOffset(n1, joint) * rotOffset,
	  rot2 = PhysicsConstraint::getRotationOffset(n2, joint) * rotOffset;
	Vector3 trans1 = PhysicsConstraint::getTranslationOffset(n1, joint),
	  trans2 = PhysicsConstraint::getTranslationOffset(n2, joint);
	return addConstraint(n1, n2, id, type, rot1, trans1, rot2, trans2, parentChild);
}

PhysicsConstraint* T4TApp::addConstraint(MyNode *n1, MyNode *n2, int id, const char *type,
  Quaternion &rot1, Vector3 &trans1, Quaternion &rot2, Vector3 &trans2, bool parentChild) {
	MyNode *node[2];
	PhysicsRigidBody *body[2];
	Vector3 trans[2];
	Quaternion rot[2];
	PhysicsConstraint *ret;
	unsigned short i, j;
	for(i = 0; i < 2; i++) {
		node[i] = i == 0 ? n1 : n2;
		body[i] = node[i]->getCollisionObject()->asRigidBody();
		rot[i] = i == 0 ? rot1 : rot2;
		trans[i] = i == 0 ? trans1 : trans2;
		/*if(strcmp(type, "hinge") == 0) {
			body[i]->setEnabled(false);
			body[i]->setFriction(0.01f);
			body[i]->setEnabled(true);
		}//*/
	}
	if(strcmp(type, "hinge") == 0) {
		ret = getPhysicsController()->createHingeConstraint(body[0], rot[0], trans[0], body[1], rot[1], trans[1]);
	} else if(strcmp(type, "spring") == 0) {
		ret = getPhysicsController()->createSpringConstraint(body[0], rot[0], trans[0], body[1], rot[1], trans[1]);
	} else if(strcmp(type, "socket") == 0) {
		ret = getPhysicsController()->createSocketConstraint(body[0], trans[0], body[1], trans[1]);
	} else if(strcmp(type, "fixed") == 0) {
		ret = getPhysicsController()->createFixedConstraint(body[0], body[1]);
	}
	bool append = id < 0;
	if(id < 0) id = _constraintCount++;
	nodeConstraint *constraint;
	for(i = 0; i < 2; i++) {
		if(append) {
			constraint = new nodeConstraint();
			node[i]->_constraints.push_back(constraint);
		}
		else {
			for(j = 0; j < node[i]->_constraints.size() && node[i]->_constraints[j]->id != id; j++);
			constraint = node[i]->_constraints[j];
		}
		constraint->other = node[(i+1)%2]->getId();
		constraint->type = type;
		constraint->rotation = rot[i];
		constraint->translation = trans[i];
		constraint->id = id;
		constraint->isChild = parentChild && i == 1;
	}
	_constraints[id] = ConstraintPtr(ret);
	if(parentChild) {
		n1->addChild(n2);
		n2->_constraintParent = n1;
		n2->_constraintId = id;
		n2->_parentOffset = trans[0];
		Matrix m;
		Matrix::createRotation(rot[0], &m);
		m.transformVector(Vector3::unitZ(), &n2->_parentAxis);
	}
	return ret;
}

void T4TApp::addConstraints(MyNode *node) {
	nodeConstraint *c1, *c2;
	unsigned short i, j;
	for(i = 0; i < node->_constraints.size(); i++) {
		c1 = node->_constraints[i];
		if(c1->id >= 0) continue;
		MyNode *other = dynamic_cast<MyNode*>(_activeScene->findNode(c1->other.c_str()));
		if(!other || !other->getCollisionObject()) continue;
		for(j = 0; j < other->_constraints.size(); j++) {
			c2 = other->_constraints[j];
			if(c2->other.compare(node->getId()) == 0 && c2->type.compare(c1->type) == 0 && c2->id < 0) {
				c1->id = _constraintCount;
				c2->id = _constraintCount++;
				addConstraint(node, other, c1->id, c1->type.c_str(), c1->rotation, c1->translation,
					c2->rotation, c2->translation, c2->isChild);
			}
		}
	}
}

void T4TApp::removeConstraints(MyNode *node, MyNode *otherNode, bool erase) {
	PhysicsController *controller = getPhysicsController();
	nodeConstraint *c1, *c2;
	unsigned short i, j;
	for(i = 0; i < node->_constraints.size(); i++) {
		c1 = node->_constraints[i];
		if(c1->id < 0) continue;
		MyNode *other = dynamic_cast<MyNode*>(_activeScene->findNode(c1->other.c_str()));
		if(!other || (otherNode && other != otherNode)) continue;
		for(j = 0; j < other->_constraints.size(); j++) {
			c2 = other->_constraints[j];
			if(c2->other.compare(node->getId()) == 0 && c2->type.compare(c1->type) == 0 && c2->id == c1->id) {
				_constraints.erase(c1->id);
				if(erase) {
					node->_constraints.erase(node->_constraints.begin() + i);
					other->_constraints.erase(other->_constraints.begin() + j);
				} else {
					c1->id = -1;
					c2->id = -1;
				}
			}
		}
	}
}

void T4TApp::enableConstraints(MyNode *node, bool enable) {
	int id;
	for(short i = 0; i < node->_constraints.size(); i++) {
		nodeConstraint *constraint = node->_constraints[i];
		id = constraint->id;
		if(id < 0 || _constraints.find(id) == _constraints.end() || (!enable && !_constraints[id]->isEnabled())) continue;
		_constraints[id]->setEnabled(enable);
		if(!enable) cout << "disabled constraint " << id << " between " << node->getId() << " and " << constraint->other << endl;
	}
}

void T4TApp::reloadConstraint(MyNode *node, nodeConstraint *constraint) {
	int id = constraint->id;
	if(id >= 0 && _constraints.find(id) != _constraints.end()) {
		_constraints.erase(id);
	}
	MyNode *other = dynamic_cast<MyNode*>(_activeScene->findNode(constraint->other.c_str()));
	if(!other) return;
	nodeConstraint *otherConstraint;
	for(short i = 0; i < other->_constraints.size(); i++) {
		otherConstraint = other->_constraints[i];
		if(otherConstraint->id == id) {
			addConstraint(node, other, id, constraint->type.c_str(), constraint->rotation, constraint->translation,
			  otherConstraint->rotation, otherConstraint->translation, constraint->isChild || otherConstraint->isChild);
			break;
		}
	}
}


T4TApp::HitFilter::HitFilter(T4TApp *app_) : app(app_) {}

bool T4TApp::HitFilter::filter(PhysicsCollisionObject *object) {
	if(object == NULL) return true;
	MyNode *node = dynamic_cast<MyNode*>(object->getNode());
	if(!node || app->auxNode(node)) return true;
}


T4TApp::NodeFilter::NodeFilter() : _node(NULL) {}

void T4TApp::NodeFilter::setNode(MyNode *node) {
	_node = node;
}

bool T4TApp::NodeFilter::filter(PhysicsCollisionObject *object) {
	return object->getNode() != _node;
}


MenuFilter::MenuFilter(Container *container) : _container(container) {
	app = (T4TApp*) Game::getInstance();
	_ordered = _container->getControls();
	_filtered = Container::create("filtered", app->_theme->getStyle("basicContainer"));
}

void MenuFilter::filter(const char *id, bool filter) {
	if(filter) {
		Control *control = _container->getControl(id);
		if(control) _filtered->addControl(control);
	} else {
		Control *control = _filtered->getControl(id);
		if(control) {
			//preserve the original button order when making this one visible again
			short position = 0;
			std::vector<Control*>::const_iterator it;
			for(it = _ordered.begin(); *it != control && it != _ordered.end(); it++) {
				if(_container->getControl((*it)->getId()) == *it) position++;
			}
			if(it == _ordered.end() || position >= _container->getControls().size()) _container->addControl(control);
			else _container->insertControl(control, position);
		}
	}
}

void MenuFilter::filterAll(bool filter) {
	std::vector<Control*>::const_iterator it;
	for(it = _ordered.begin(); it != _ordered.end(); it++) {
		this->filter((*it)->getId(), filter);
	}
}


void T4TApp::setAction(const char *type, ...) {

	va_list args;
	va_start(args, type);

	if(_action == NULL) _action = new Action();
	Action *action = _action;
	action->type = type;
	short n, i, j = action->refNodes.size();
	if(strcmp(type, "constraint") == 0) n = 2;
	else n = 1;
	action->nodes.resize(n);
	action->refNodes.resize(n);
	for(i = 0; i < n; i++) {
		action->nodes[i] = (MyNode*) va_arg(args, MyNode*);
		if(i >= j) action->refNodes[i] = MyNode::create("ref");
	}
	MyNode *node = action->nodes[0], *ref = action->refNodes[0];

	if(strcmp(type, "addNode") == 0) {
	} else if(strcmp(type, "deleteNode") == 0) {
	} else if(strcmp(type, "position") == 0) {
		ref->set(node);
	} else if(strcmp(type, "constraint") == 0) {
		nodeConstraint *constraint = new nodeConstraint();
		constraint->id = _constraintCount;
		for(i = 0; i < 2; i++) {
			node = action->nodes[i];
			ref = action->refNodes[i];
			ref->set(node);
			ref->_constraints.push_back(constraint);
		}
	} else if(strcmp(type, "tool") == 0) {
		ref->copyMesh(node);
	} else if(strcmp(type, "test") == 0) {
		ref->set(node);
	}
	return;
}

void T4TApp::commitAction() {
	if(_action == NULL) return;
	delAll(_undone); //can't redo anything once something else is done
	_history.push_back(_action);
	_action = NULL;
	_undo->setEnabled(true);
	_redo->setEnabled(false);
}

void T4TApp::undoLastAction() {
	if(_history.empty()) return;
	Action *action = popBack(_history);
	const char *type = action->type.c_str();
	MyNode *node = action->nodes[0], *ref = action->refNodes[0];
	bool allowRedo = true;

	if(strcmp(type, "addNode") == 0) {
		removeNode(node, false);
	} else if(strcmp(type, "deleteNode") == 0) {
		_scene->addNode(node);
		node->enablePhysics();
		node->release();
	} else if(strcmp(type, "position") == 0) {
		swapTransform(node, ref);
	} else if(strcmp(type, "constraint") == 0) {
		nodeConstraint *constraint[2];
		short i;
		for(i = 0; i < 2; i++) constraint[i] = action->nodes[i]->_constraints.back();
		if(constraint[0]->id < 0 || constraint[0]->id != constraint[1]->id) {
			GP_WARN("ID mismatch undoing constraint: %d <> %d", constraint[0]->id, constraint[1]->id);
			allowRedo = false;
		} else {
			int id = constraint[0]->id;
			if(_constraints.find(id) == _constraints.end()) {
				GP_WARN("No constraint with id %d", id);
				allowRedo = false;
			} else {
				_constraints.erase(id);
				for(i = 0; i < 2; i++) {
					node = action->nodes[i];
					ref = action->refNodes[i];
					ref->_constraints.push_back(popBack(node->_constraints));
					if(i == 1) {
						_scene->addNode(node); //also removes it from its constraint parent
						node->_constraintParent = NULL;
						swapTransform(node, ref);
					}
				}
			}
		}
	} else if(strcmp(type, "tool") == 0) {
		swapMesh(node, ref);
		node->updateModel();
	} else if(strcmp(type, "test") == 0) {
		removeNode(node);
	}
	if(allowRedo) {
		_undone.push_back(action);
		_redo->setEnabled(true);
	}
	if(_history.empty()) _undo->setEnabled(false);
}

void T4TApp::redoLastAction() {
	if(_undone.empty()) return;
	Action *action = popBack(_undone);
	const char *type = action->type.c_str();
	MyNode *node = action->nodes[0], *ref = action->refNodes[0];

	if(strcmp(type, "addNode") == 0) {
		_scene->addNode(node);
		node->enablePhysics();
		node->release();
	} else if(strcmp(type, "deleteNode") == 0) {
		removeNode(node, false);
	} else if(strcmp(type, "position") == 0) {
		swapTransform(node, ref);
	} else if(strcmp(type, "constraint") == 0) {
		std::string type;
		Quaternion rot[2];
		Vector3 trans[2];
		bool parentChild = false;
		for(short i = 0; i < 2; i++) {
			//put the node back in its constraint-friendly position
			node = action->nodes[i];
			ref = action->refNodes[i];
			swapTransform(node, ref);
			//retrieve each side of the constraint from the reference node
			nodeConstraint *constraint = popBack(ref->_constraints);
			type = constraint->type;
			rot[i] = constraint->rotation;
			trans[i] = constraint->translation;
			parentChild = parentChild || constraint->isChild;
			delete constraint;
		}
		addConstraint(action->nodes[0], action->nodes[1], -1, type.c_str(), rot[0], trans[0], rot[1], trans[1], parentChild);
		action->nodes[0]->addChild(action->nodes[1]);
		action->nodes[1]->_constraintParent = action->nodes[0];
	} else if(strcmp(type, "tool") == 0) {
		swapMesh(node, ref);
		node->updateModel();
	} else if(strcmp(type, "test") == 0) {
		action->nodes[0] = dropBall(ref->getTranslationWorld());
	}
	_history.push_back(action);
	_undo->setEnabled(true);
	if(_undone.empty()) _redo->setEnabled(false);
}

void T4TApp::swapTransform(MyNode *n1, MyNode *n2) {
	_tmpNode->set(n1);
	n1->set(n2);
	n2->set(_tmpNode);
}

void T4TApp::swapMesh(MyNode *n1, MyNode *n2) {
	_tmpNode->copyMesh(n1);
	n1->copyMesh(n2);
	n2->copyMesh(_tmpNode);
}


/************* MISCELLANEOUS ****************/
template <class T> T* T4TApp::popBack(std::vector<T*> &vec) {
	if(vec.empty()) return NULL;
	T* t = vec.back();
	vec.pop_back();
	return t;
}

template <class T> void T4TApp::delBack(std::vector<T*> &vec) {
	if(vec.empty()) return NULL;
	T* t = vec.back();
	vec.pop_back();
	delete t;
}

template <class T> void T4TApp::delAll(std::vector<T*> &vec) {
	for(typename std::vector<T*>::iterator it = vec.begin(); it != vec.end(); it++) {
		T* t = *it;
		delete t;
	}
	vec.clear();
}

}

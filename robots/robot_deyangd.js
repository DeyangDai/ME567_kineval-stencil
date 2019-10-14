//   CREATE ROBOT STRUCTURE

// KE 

links_geom_imported = false;

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

// create robot data object
robot = new Object(); // or just {} will create new object

// give the robot a name
robot.name = "tank";

// initialize start pose of robot in the world
robot.origin = {xyz: [0,0.3,0], rpy:[0,0,0]};

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "body";

        
// specify and create data objects for the links of the robot
robot.links = {
    "body": {},
    "wheel_front_left": {},
    "wheel_front_right": {} ,
    "wheel_back_left": {},
    "wheel_back_right": {},
    "gun_bore": {},
    "chamber": {}
};

//////////////////////////////////////////////////
/////     DEFINE JOINTS AND KINEMATIC HIERARCHY
//////////////////////////////////////////////////

/*      joint definition template
        // specify parent/inboard link and child/outboard link
        robot.joints.joint1 = {parent:"link1", child:"link2"};
        // joint origin's offset transform from parent link origin
        robot.joints.joint1.origin = {xyz: [5,3,0], rpy:[0,0,0]}; 
        // joint rotation axis
        robot.joints.joint1.axis = [0.0,0.0,1.0]; 
*/


// roll-pitch-yaw defined by ROS as corresponding to x-y-z 
//http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file

// specify and create data objects for the joints of the robot
robot.joints = {};

robot.joints.body_wheel_front_left = {parent:"body", child:"wheel_front_left"};
robot.joints.body_wheel_front_left.origin = {xyz: [0.8,0,0.6], rpy:[Math.PI / 2,0,Math.PI / 2]};
robot.joints.body_wheel_front_left.axis = [0.0,0.0,-1.0];

robot.joints.body_wheel_front_right = {parent:"body", child:"wheel_front_right"};
robot.joints.body_wheel_front_right.origin = {xyz: [-0.8,0,0.6], rpy:[Math.PI / 2,0,Math.PI / 2]};
robot.joints.body_wheel_front_right.axis = [0.0,0.0,-1.0];

robot.joints.body_wheel_back_left = {parent:"body", child:"wheel_back_left"};
robot.joints.body_wheel_back_left.origin = {xyz: [0.8,0,-0.6], rpy:[Math.PI / 2,0,Math.PI / 2]};
robot.joints.body_wheel_back_left.axis = [0.0,0.0,-1.0];

robot.joints.body_wheel_back_right = {parent:"body", child:"wheel_back_right"};
robot.joints.body_wheel_back_right.origin = {xyz: [-0.8,0,-0.6], rpy:[Math.PI / 2,0,Math.PI / 2]};
robot.joints.body_wheel_back_right.axis = [0.0,0.0,-1.0];

robot.joints.body_chamber = {parent:"body", child:"chamber"};
robot.joints.body_chamber.origin = {xyz: [0,0.4,0], rpy:[0,0,0]};
robot.joints.body_chamber.axis = [0.0,1.0,0.0];

robot.joints.chamber_gun_bore = {parent:"chamber", child:"gun_bore"};
robot.joints.chamber_gun_bore.origin = {xyz: [0,0.2,0.5], rpy:[Math.PI / 2.3, 0, 0]};
robot.joints.chamber_gun_bore.axis = [0.0,1.0,0.0];

// // specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "chamber_gun_bore";
robot.endeffector.position = [[0],[0],[0.5],[1]];

//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////

/*  threejs geometry definition template, will be used by THREE.Mesh() to create threejs object
    // create threejs geometry and insert into links_geom data object
    links_geom["link1"] = new THREE.CubeGeometry( 5+2, 2, 2 );

    // example of translating geometry (in object space)
    links_geom["link1"].applyMatrix( new THREE.Matrix4().makeTranslation(5/2, 0, 0) );

    // example of rotating geometry 45 degrees about y-axis (in object space)
    var temp3axis = new THREE.Vector3(0,1,0);
    links_geom["link1"].rotateOnAxis(temp3axis,Math.PI/4);
*/

// define threejs geometries and associate with robot links 
links_geom = {};

links_geom["body"] = new THREE.CubeGeometry( 1.5, 0.5, 2 );
links_geom["body"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0.2, 0) );

links_geom["wheel_front_left"] = new THREE.CylinderGeometry(0.3, 0.3, 0.2);
links_geom["wheel_front_left"].applyMatrix( new THREE.Matrix4().makeRotationX(Math.PI / 2) );

links_geom["wheel_front_right"] = new THREE.CylinderGeometry(0.3, 0.3, 0.2);
links_geom["wheel_front_right"].applyMatrix( new THREE.Matrix4().makeRotationX(Math.PI / 2) );

links_geom["wheel_back_left"] = new THREE.CylinderGeometry(0.3, 0.3, 0.2);
links_geom["wheel_back_left"].applyMatrix( new THREE.Matrix4().makeRotationX(Math.PI / 2) );

links_geom["wheel_back_right"] = new THREE.CylinderGeometry(0.3, 0.3, 0.2);
links_geom["wheel_back_right"].applyMatrix( new THREE.Matrix4().makeRotationX(Math.PI / 2) );

links_geom["chamber"] = new THREE.CubeGeometry( 0.8, 0.3, 1.2 );
links_geom["chamber"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0.2, 0) );

links_geom["gun_bore"] = new THREE.CylinderGeometry(0.06, 0.06, 2);
links_geom["gun_bore"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0.2, 0) );
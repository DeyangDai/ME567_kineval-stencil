
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: implement kineval.buildFKTransforms();
    kineval.buildFKTransforms();
};

    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //

kineval.buildFKTransforms = function buildFKTransforms() {
    mstack = [generate_identity()];
    traverseFKBase();
};

function traverseFKBase() {
    var T = generate_transformation(robot.origin.xyz, robot.origin.rpy);
    robot_heading = matrix_multiply(T, [0, 0, 1, 1]);
    robot_lateral = matrix_multiply(T, [1, 0, 0, 1]);

    if (robot.links_geom_imported === true) {
        var T_ros2threejs = matrix_multiply(
            generate_rotation_matrix_Y(-Math.PI / 2),
            generate_rotation_matrix_X(-Math.PI / 2));
        mstack.push(matrix_multiply(T, T_ros2threejs));
    } else {
        mstack.push(T);
    }

    var base = robot.links[robot.base];
    base.xform = mstack[mstack.length - 1];

    if (typeof base.children !== 'undefined') {
        for (var i = 0; i < base.children.length; i++) {
            traverseFKJoint(base.children[i]);
        }
    }
    mstack.pop();
}

function traverseFKLink(link_name) {
    var link = robot.links[link_name];
    link.xform = mstack[mstack.length - 1];

    if (typeof link.children !== 'undefined') {
        for (var i = 0; i < link.children.length; i++) {
            traverseFKJoint(link.children[i]);
        }
    }
}

function traverseFKJoint(joint_name) {
    var joint = robot.joints[joint_name];
    var T = generate_transformation(joint.origin.xyz, joint.origin.rpy);
    var xform = matrix_multiply(mstack[mstack.length - 1], T);

    mstack.push(xform);
    joint.xform = mstack[mstack.length - 1];
    traverseFKLink(joint.child);
    mstack.pop();
}
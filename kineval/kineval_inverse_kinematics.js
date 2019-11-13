
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
};

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
    kineval.params.trial_ik_random.start.getTime();

    // get endeffector Cartesian position in the world
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);
    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
        Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
        + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
        + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );
    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
        kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
        kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
        kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
        kineval.params.trial_ik_random.targets += 1;
        textbar.innerHTML = "IK trial Random: target "
            + kineval.params.trial_ik_random.targets
            + " reached at time "
            + kineval.params.trial_ik_random.time;
    }
};

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration
    var kinematics_chain = [];
    var curr_joint = endeffector_joint;
    while (robot.joints[curr_joint].parent !== robot.base) {
        kinematics_chain.push(curr_joint);
        curr_joint = robot.links[robot.joints[curr_joint].parent].parent;
    }
    kinematics_chain.push(curr_joint);

    var J = [[], [], [], [], [], []]; // Jacobian matrix
    var endeffector_position_world = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_position_local);

    for (var i = 0; i < kinematics_chain.length; i++) {
        // var j = kinematics_chain.length - i - 1;
        var joint = robot.joints[kinematics_chain[i]];
        var joint_origin_world = matrix_multiply(joint.xform, [[0], [0], [0], [1]]);

        var axis_local = [[joint.axis[0]], [joint.axis[1]], [joint.axis[2]], [0]];
        var axis_world = matrix_multiply(joint.xform, axis_local);

        var J_cross = vector_cross(axis_world, vector_substract(endeffector_position_world, joint_origin_world));
        for (var k = 0; k < 3; k++) {
            if (joint.type === 'prismatic') {
                J[k][i] = axis_world[k][0];
                J[k + 3][i] = 0;
            } else {
                J[k][i] = J_cross[k];
                J[k + 3][i] = axis_world[k][0];
            }
        }
    }

    var endeffector_orientation_world = rotation_matrix_to_axisangle(robot.joints[endeffector_joint].xform);
    var delta_position = vector_substract(endeffector_target_world.position, endeffector_position_world);
    var delta_orientation = vector_substract(endeffector_target_world.orientation, endeffector_orientation_world);
    var delta_x = [];
    for (i = 0; i < 3; i++) {
        delta_x[i] = [delta_position[i]];
        if (kineval.params.ik_orientation_included) {
            delta_x[i + 3] = [delta_orientation[i]];
        } else {
            delta_x[i + 3] = [0];
        }
    }

    var delta_q;
    if (kineval.params.ik_pseudoinverse) {
        // var pseudoinverse = numeric.inv(matrix_multiply(J, matrix_transpose(J)));
        // delta_q = matrix_multiply(matrix_multiply(matrix_transpose(J), pseudoinverse), delta_x);
        var pseudoinverse = matrix_pseudoinverse(J);
        delta_q = matrix_multiply(pseudoinverse, delta_x);
    } else {
        delta_q = matrix_multiply(matrix_transpose(J), delta_x);
    }

    for (i = 0; i < kinematics_chain.length; i++) {
        robot.joints[kinematics_chain[i]].control += kineval.params.ik_steplength * delta_q[i][0];
    }
};




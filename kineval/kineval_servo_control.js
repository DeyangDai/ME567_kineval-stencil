
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return; 

    // STENCIL: implement FSM to cycle through dance pose setpoints
    var currTime = new Date();
    console.log('curr: ' + currTime);
    console.log('last: ' + lastTime);
    if ((robot.name === 'fetch' && currTime - lastTime > 250) || (robot.name === 'baxter' && currTime - lastTime > 500)) {
        kineval.params.dance_pose_index = kineval.params.dance_sequence_index.shift();
        kineval.params.dance_sequence_index.push(kineval.params.dance_pose_index);
        for (var i in robot.joints) {
            kineval.params.setpoint_target[i] = kineval.setpoints[kineval.params.dance_pose_index][i];
        }
        lastTime = currTime;
    }
}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints
    for (i in robot.joints) {
        var joint = robot.joints[i];
        joint.control = (kineval.params.setpoint_target[i] - joint.angle) * joint.servo.p_gain;
    }
}



#include "unitree_arm_sdk/control/unitreeArm.h"

using namespace UNITREE_ARM;

int main(int argc, char *argv[]) {
    std::cout << std::fixed << std::setprecision(3);
    bool hasGripper = true;
    unitreeArm arm(hasGripper);
    arm.sendRecvThread->start(); // arm.sendRecvThread->start(); starts a background loop thread that repeatedly calls unitreeArm::sendRecv() at a fixed rate (comment says 500 Hz) to exchange UDP messages with the Z1 controller. This keeps the arm’s state (lowstate) updated and pushes commands (lowcmd) continuously; many examples rely on this thread being active during control and then call shutdown() when done.

    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);  // Finite state machine state set to PASSIVE, allowing for
                                       // low-level command control.
    arm.setFsm(ArmFSMState::LOWCMD); // FSM in z1_sdk is the arm’s finite state machine, i.e., the control mode the controller is currently in and the mode you command it to switch to. It’s defined as ArmFSMState and used by APIs like setFsm() and startTrack().

    std::vector<double> KP, KW;
    KP = arm._ctrlComp->lowcmd->kp;
    KW = arm._ctrlComp->lowcmd->kd;  // k_d
    arm._ctrlComp->lowcmd->setControlGain(KP, KW);
    // arm._ctrlComp->lowcmd->setGripperGain(KP[KP.size()-1], KW[KW.size()-1]);
    arm.sendRecvThread->shutdown();

    Vec6 initQ = arm.lowstate->getQ();

    double duration = 1000;
    Vec6 targetQ;
    targetQ << 0, 1.5, -1, -0.54, 0, 0;
    Timer timer(arm._ctrlComp->dt); // set frequency to control loop dt
    // below: mannually send commmand instead of using sendRecvThread
    for(int i(0); i<duration; i++){ // iterate over the trajectory samples
        arm.q = initQ * (1-i/duration) + targetQ * (i/duration); // linearly interpolate joint positions
        arm.qd = (targetQ - initQ) / (duration * arm._ctrlComp->dt); // constant joint velocity for this segment
        arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero()); // compute feedforward torques
        arm.gripperQ = -(i/duration); // linearly close the gripper over the motion
        
        arm.setArmCmd(arm.q, arm.qd, arm.tau); // write joint commands into lowcmd
        arm.setGripperCmd(arm.gripperQ, arm.gripperW, arm.gripperTau); // write gripper commands into lowcmd
        arm.sendRecv(); // send commands and receive state once per step
        timer.sleep(); // keep the loop period at dt
    }

    arm.sendRecvThread->start(); // resume background send/receive loop
    arm.setFsm(ArmFSMState::JOINTCTRL); // switch to joint control mode for the return motion
    arm.backToStart(); // move the arm back to the home position
    arm.setFsm(ArmFSMState::PASSIVE); // relax the arm after motion completes
    arm.sendRecvThread->shutdown(); // stop the background send/receive loop
    return 0; // exit program
}

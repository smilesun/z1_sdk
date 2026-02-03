#include "unitree_arm_sdk/control/unitreeArm.h"
#include <fstream>
#include <chrono>
#include <algorithm>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

using namespace UNITREE_ARM;

// Clamp each element of a 6D vector to [low, high] for torque limiting.
Vec6 clampVec6(const Vec6& v, double low, double high)
{
    Vec6 result;
    for (int i = 0; i < 6; i++) {
        result[i] = std::max(low, std::min(v[i], high));
    }
    return result;
}

int main(int argc, char *argv[]) {
    // Open a non-blocking "barrier" file to gate the start of the motion.
    int fd = open("/tmp/run_barrier", O_RDONLY | O_NONBLOCK); //open a temporary file for busy waiting
    //return -1 with errno = EAGAIN/EWOULDBLOCK if no data is available, so the loop can keep running without getting stuck.


    // Load the nominal trajectory CSV from disk.
    using clock = std::chrono::steady_clock;
    std::ifstream file("../examples/nominal_trajectory.csv");
    if (!file.is_open()) {
        std::cerr << "Error opening file\n";
        return 1;
    }

    // Parse CSV rows into a vector of doubles.
    std::string line;
    std::getline(file, line); // skip header

    std::vector<std::vector<double>> rows;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> row;
        while (std::getline(ss, cell, ',')) {
            row.push_back(std::stod(cell));
        }
        rows.push_back(row);
    }

    // Convert to Eigen matrices for convenient column access.
    size_t nrows = rows.size();
    size_t ncols = rows[0].size();
    Eigen::MatrixXd data(nrows, ncols);
    for (size_t i = 0; i < nrows; ++i)
        for (size_t j = 0; j < ncols; ++j)
            data(i, j) = rows[i][j];

    // Extract time, joint positions, velocities, and feedforward torques.
    Eigen::VectorXd t_interp = data.col(0);
    Eigen::MatrixXd q_interp = data.block(0, 1, nrows, 6).transpose();
    Eigen::MatrixXd q_dot_interp = data.block(0, 7, nrows, 6).transpose();
    Eigen::MatrixXd tau_interp = data.block(0, 13, nrows, 6).transpose();

    // std::cout << "Loaded " << nrows << " rows.\n";
    // std::cout << "First time: " << t(0) << "\n";
    // std::cout << "First q: " << q.col(0).transpose() << "\n";
    // Determine the trajectory sample period and endpoint poses.
    double dt = t_interp(1) - t_interp(0);
    Vec6 q_init = q_interp.col(0);
    Vec6 q_end = q_interp.col(q_interp.cols() - 1);

    // Initialize arm interface and start background send/receive thread.
    std::cout << std::fixed << std::setprecision(3);
    bool hasGripper = true;
    unitreeArm arm(hasGripper);
    arm.sendRecvThread->start();

    // Move to home, then switch to low-level command mode.
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.setFsm(ArmFSMState::LOWCMD);

    // Read default gains and re-apply them explicitly.
    std::vector<double> KP, KW;
    KP = arm._ctrlComp->lowcmd->kp;
    KW = arm._ctrlComp->lowcmd->kd;

    // std::vector<double> KP = {20, 30, 30, 20, 15, 10};
    // std::vector<double> KW = {2000, 2000, 2000, 2000, 2000, 2000};
    arm._ctrlComp->lowcmd->setControlGain(KP, KW);    
    Eigen::Map<const Vec6> KP_Eigen(KP.data());
    Eigen::Map<const Vec6> KW_Eigen(KW.data());
    // arm._ctrlComp->lowcmd->setGripperGain(KP[KP.size()-1], KW[KW.size()-1]);
    // Stop the background thread; this example drives send/recv manually.
    arm.sendRecvThread->shutdown();

    Vec6 initQ = arm.lowstate->getQ();

    // Ramp from current pose to the initial trajectory pose.
    double duration;
    duration = 3/arm._ctrlComp->dt; // use 3 seconds to reach the initial pose
    double tau_limit = 30.0;
    Vec6 targetQ;
    targetQ << q_init;
    Vec6 outputTau;
    Timer timer_init(arm._ctrlComp->dt);
    Timer timer(dt);
    for(int i(0); i<duration; i++){
        arm.q = initQ * (1-i/duration) + targetQ * (i/duration);
        arm.qd = (targetQ - initQ) / (duration * arm._ctrlComp->dt);
        outputTau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
        arm.tau = clampVec6(outputTau, -tau_limit, tau_limit);
        arm.gripperQ = 0;
        // std::cout << arm.tau.transpose() << "\n";
        std::cout << arm.q.transpose()<<"\n" << arm.lowstate->getQ().transpose() << "difference:"<< (arm.q-arm.lowstate->getQ()).transpose() <<"\n";

        arm.setArmCmd(arm.q, arm.qd, arm.tau);
        arm.setGripperCmd(arm.gripperQ, arm.gripperW, arm.gripperTau);
        arm.sendRecv();
        timer_init.sleep();
    }

    // Hold at the initial pose until an external signal is received.
    while (true) //std::chrono::duration<double>(clock::now() - start_time).count() < warmup_time
    {
        arm.q << q_init;
        arm.qd << 0,0,0,0,0,0;
        outputTau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
        arm.tau = clampVec6(outputTau, -tau_limit, tau_limit);
        arm.gripperQ = 0;
        // std::cout << arm.tau.transpose()<<"\n";
        
        arm.setArmCmd(arm.q, arm.qd, arm.tau);
        arm.setGripperCmd(arm.gripperQ, arm.gripperW, arm.gripperTau);
        arm.sendRecv();

        char buf;
        // try to read, if no data, read will immediately return -1
        if (read(fd, &buf, 1) > 0) {
            std::cout << "Signal received, continue!" << std::endl;
            break;
        }

        timer.sleep();
    }

    // Optional fixed delay before executing the catch trajectory.
    double time_delay = 0.121;  // 0.12 seconds for robust nominal trajectory, 0.125 seconds for robust nominal trajectory
    auto start_time = clock::now();
    while (std::chrono::duration<double>(clock::now() - start_time).count() < time_delay) //
    {
        arm.q << q_init;
        arm.qd << 0,0,0,0,0,0;
        outputTau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
        // - qdd: joint accelerations, set to zero (no acceleration assumed) 
        // - Ftip: end‑effector spatial force, set to zero (no external wrench)

        arm.tau = clampVec6(outputTau, -tau_limit, tau_limit);
        arm.gripperQ = 0;
        // std::cout << arm.tau.transpose()<<"\n";
        
        arm.setArmCmd(arm.q, arm.qd, arm.tau);
        arm.setGripperCmd(arm.gripperQ, arm.gripperW, arm.gripperTau);
        arm.sendRecv();

        timer.sleep();
    }

    // Execute the trajectory from the CSV.
    auto actual_delay = std::chrono::duration<double>(clock::now() - start_time);
    std::cout << "Actual time delay: " << actual_delay.count()<<"\n";

    int catching_steps = t_interp.size();
    Vec6 currentQ;
    Vec6 currentQd;
    Vec6 currentTau;
    // wait_for_go();
    for(int i(0); i<catching_steps; i++){
        arm.q = q_interp.col(i);
        arm.qd = q_dot_interp.col(i);
        outputTau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());//<< tau_interp.col(i);
        currentQ = arm.lowstate->getQ();
        currentQd = arm.lowstate->getQd();
        arm.tau = clampVec6(outputTau, -tau_limit, tau_limit);
        arm.gripperQ = 0;

        currentTau = arm.lowstate->getTau();
        // std::cout << arm.tau.transpose()<<"\n";
        // std::cout << currentTau.transpose() << "and\t"<< arm.tau.transpose()<< "\n";
        // std::cout << currentTau.transpose() << "taucmd:\n"<< arm.tau.transpose()<< "\n";
        // std::cout << arm.q.transpose()<<"\n" << currentQ.transpose() << "difference:"<< (arm.q-currentQ).transpose() <<"\n";
        
        arm.setArmCmd(arm.q, arm.qd, arm.tau);
        arm.setGripperCmd(arm.gripperQ, arm.gripperW, arm.gripperTau);
        arm.sendRecv();
        timer.sleep();
    }

    // Hold the final pose for a fixed duration.
    double holding_time = 10.0;  // seconds
    auto finish_time = clock::now();
    while (std::chrono::duration<double>(clock::now() - finish_time).count() < holding_time) 
    {
        arm.q << q_end;
        arm.qd << 0,0,0,0,0,0;
        outputTau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
        arm.tau = clampVec6(outputTau, -tau_limit, tau_limit);
        arm.gripperQ = 0;
        // std::cout << arm.tau.transpose()<<"\n";
        
        arm.setArmCmd(arm.q, arm.qd, arm.tau);
        arm.setGripperCmd(arm.gripperQ, arm.gripperW, arm.gripperTau);
        arm.sendRecv();

        timer.sleep();
    }

    // Restart the background thread for standard post-motion handling.
    arm.sendRecvThread->start();

    arm.setFsm(ArmFSMState::JOINTCTRL);
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();

    // Close the barrier file and exit.
    close(fd);
    return 0;
}

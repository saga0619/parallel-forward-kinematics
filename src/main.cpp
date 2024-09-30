#include <iostream>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <Eigen/Dense>
#include <random>
#include <chrono>
#include <getopt.h>
#include <thread>

#ifndef URDF_DIR
#define URDF_DIR ""
#endif

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

void print_help(const char *program_name)
{
    std::cout << "Usage: " << program_name << " [Option]\n"
              << "Option:\n"
              << "  -u <urdf file path> : urdf path \n"
              << "  -n <interation>     : iteration (default : 10000)\n"
              << "  -t <thread count>   : thread count (default : 1)\n"
              << "  -v                  : verbose \n"
              << "  -h                  : help \n";
}

void run_iteration(Model &model, unsigned int num_iterations, int thread_id)
{
    RigidBodyDynamics::Model model_local = model;

    // print current pid 
    // std::cout << "thread id : " << thread_id << " : start" << std::endl;
    unsigned int dof = model_local.dof_count;

    // Panda 로봇의 관절 한계 (라디안)
    VectorNd joint_limits_lower(dof);
    VectorNd joint_limits_upper(dof);

    joint_limits_lower.setConstant(-M_PI);
    joint_limits_upper.setConstant(M_PI);

    std::random_device rd;
    std::mt19937 gen(rd());

    std::vector<std::uniform_real_distribution<double>> distributions;
    for (unsigned int i = 0; i < dof; ++i)
    {
        distributions.emplace_back(joint_limits_lower[i], joint_limits_upper[i]);
    }

    const char *end_effector_name = "panda_hand";

    unsigned int end_effector_id = model_local.GetBodyId(end_effector_name);
    if (end_effector_id == std::numeric_limits<unsigned int>::max())
    {
        std::cerr << "cannot find end effector " << end_effector_name << std::endl;
        return;
    }

    Vector3d end_effector_point = Vector3d::Zero();

    Vector3d end_effector_position;

    for (unsigned int iter = 0; iter < num_iterations; ++iter)
    {
        //generate random joint angles
        VectorNd Q(dof);
        for (unsigned int i = 0; i < dof; ++i)
        {
            Q[i] = distributions[i](gen);
        }

        end_effector_position = CalcBodyToBaseCoordinates(
            model_local, Q, end_effector_id, end_effector_point, true);
    }

    std::cout << "thread id : " << thread_id << " : end "<< end_effector_position.transpose() << std::endl;
}

int main(int argc, char *argv[])
{
    // 기본값 설정
    std::string urdf_filename = "panda.urdf";
    unsigned int num_iterations = 10000;
    unsigned int thread_count = 1;
    bool verbose = false;

    if (argc == 1)
    {
        print_help(argv[0]);
        return 0;
    }

    // 명령줄 옵션 파싱
    int opt;
    while ((opt = getopt(argc, argv, "u:n:t:vh")) != -1)
    {
        switch (opt)
        {
        case 'u':
            urdf_filename = optarg;
            break;
        case 'n':
            num_iterations = std::stoi(optarg);
            break;
        case 't':
            thread_count = std::stoi(optarg);
            break;
        case 'v':
            verbose = true;
            break;
        case 'h':
            print_help(argv[0]);
            return 0;
        default:
            print_help(argv[0]);
            return -1;
        }
    }

    if (verbose)
    {
        std::cout << "URDF path : " << urdf_filename << std::endl;
        std::cout << "iteration : " << num_iterations << std::endl;
        std::cout << "thread count : " << thread_count << std::endl;
    }

    Model model = Model();

    // URDF 파일로부터 모델 로드
    if (!Addons::URDFReadFromFile(urdf_filename.c_str(), &model, false))
    {
        std::cerr << "failed to load urdf." << std::endl;
        return -1;
    }

    // 모델의 자유도 수 확인
    unsigned int dof = model.dof_count;
    std::cout << "model dof : " << dof << std::endl;

    auto start_time = std::chrono::high_resolution_clock::now();

    int num_iterations_per_thread = num_iterations / thread_count;

    std::vector<std::thread> threads;
    for (unsigned int i = 0; i < thread_count; ++i)
    {
        threads.emplace_back(run_iteration, std::ref(model), num_iterations_per_thread, i);
    }

    for (auto &thread : threads)
    {
        thread.join();
    }

    auto end_time = std::chrono::high_resolution_clock::now();

    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    double elapsed_time_sec = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();

    if(elapsed_time > 1000)
        std::cout << num_iterations << " forward kinematics calculation : " << elapsed_time_sec << " s" << std::endl;
    else
        std::cout << num_iterations << " forward kinematics calculation : " << elapsed_time << " ms" << std::endl;

    return 0;
}
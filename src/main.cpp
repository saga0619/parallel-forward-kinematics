#include <iostream>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <Eigen/Dense>
#include <random>
#include <chrono>
#include <getopt.h>
#include <thread>
#include <tinyxml2.h> // TinyXML2 헤더 파일 포함

#ifndef URDF_DIR
#define URDF_DIR ""
#endif

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace tinyxml2;

void print_help(const char *program_name)
{
    std::cout << "Usage: " << program_name << " [Option]\n"
              << "Option:\n"
              << "  -u <urdf file path> : urdf path \n"
              << "  -e <end effector>   : end effector name \n"
              << "  -n <interation>     : iteration (default : 10000)\n"
              << "  -t <thread count>   : thread count (default : 1)\n"
              << "  -v                  : verbose \n"
              << "  -h                  : help \n";
}

void run_iteration(Model &model, std::string end_effector_name, VectorNd joint_limits_lower, VectorNd joint_limits_upper, unsigned int num_iterations, int thread_id)
{
    RigidBodyDynamics::Model model_local = model;

    // print current pid
    // std::cout << "thread id : " << thread_id << " : start" << std::endl;
    unsigned int dof = model_local.dof_count;

    std::random_device rd;
    std::mt19937 gen(rd());

    std::vector<std::uniform_real_distribution<double>> distributions;
    for (unsigned int i = 0; i < dof; ++i)
    {
        distributions.emplace_back(joint_limits_lower[i], joint_limits_upper[i]);
    }


    unsigned int end_effector_id = model_local.GetBodyId(end_effector_name.c_str());
    if (end_effector_id == std::numeric_limits<unsigned int>::max())
    {
        std::cerr << "cannot find end effector " << end_effector_name << std::endl;
        return;
    }

    Vector3d end_effector_point = Vector3d::Zero();

    Vector3d end_effector_position;
    Matrix3d end_effector_rotation;

    for (unsigned int iter = 0; iter < num_iterations; ++iter)
    {
        // generate random joint angles
        VectorNd Q(dof);
        for (unsigned int i = 0; i < dof; ++i)
        {
            Q[i] = distributions[i](gen);
        }

        end_effector_position = CalcBodyToBaseCoordinates(
            model_local, Q, end_effector_id, end_effector_point, true);
        end_effector_rotation = CalcBodyWorldOrientation(
            model_local, Q, end_effector_id, true);
    }

    std::cout << "thread id : " << thread_id << " : end " << end_effector_position.transpose() << std::endl;
}

int main(int argc, char *argv[])
{
    // 기본값 설정
    std::string urdf_filename = "panda.urdf";
    std::string end_effector_name = "panda_hand";
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
    while ((opt = getopt(argc, argv, "u:e:n:t:vh")) != -1)
    {
        switch (opt)
        {
        case 'u':
            urdf_filename = optarg;
            break;
        case 'e':
            end_effector_name = optarg;
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

    std::vector<std::pair<double, double>> joint_limits;

    XMLDocument urdf_doc;

    XMLError load_result = urdf_doc.LoadFile(urdf_filename.c_str());
    if (load_result != XML_SUCCESS)
    {
        std::cerr << "failed to load urdf file." << std::endl;
        return -1;
    }

    XMLElement *robot_element = urdf_doc.FirstChildElement("robot");
    if (!robot_element)
    {
        std::cerr << "failed to find robot" << std::endl;
        return -1;
    }

    for (XMLElement *joint_element = robot_element->FirstChildElement("joint");
         joint_element != nullptr;
         joint_element = joint_element->NextSiblingElement("joint"))
    {

        const char *joint_name = joint_element->Attribute("name");
        const char *joint_type = joint_element->Attribute("type");

        if (!joint_name || !joint_type)
        {
            continue;
        }

        // Revolute 또는 Continuous 관절만 처리
        if (std::string(joint_type) == "revolute" || std::string(joint_type) == "continuous")
        {
            XMLElement *limit_element = joint_element->FirstChildElement("limit");
            if (limit_element)
            {
                double lower = 0.0;
                double upper = 0.0;
                limit_element->QueryDoubleAttribute("lower", &lower);
                limit_element->QueryDoubleAttribute("upper", &upper);
                joint_limits.push_back(std::make_pair(lower, upper));
            }
        }
    }

    if (verbose)
    {
        std::cout << "joint limits" << std::endl;
        // print joint limits
        int joint_index = 0;
        for (auto &joint_limit : joint_limits)
        {
            std::cout << joint_index++ << " : " << joint_limit.first << " ~ " << joint_limit.second << std::endl;
        }
    }
    // int dof = model.dof_count;
    unsigned int dof = model.dof_count;
    if(verbose)
    {
        std::cout << "model dof : " << dof << std::endl;
        std::cout << "end effector name : " << end_effector_name << std::endl;
    }
    // 관절 한계를 저장할 벡터 생성
    VectorNd joint_limits_lower = VectorNd::Zero(dof);
    VectorNd joint_limits_upper = VectorNd::Zero(dof);

    for (unsigned int i = 0; i < dof; ++i)
    {
        joint_limits_lower[i] = joint_limits[i].first;
        joint_limits_upper[i] = joint_limits[i].second;
    }

    // XMLE`
    // URDF 파일을 다시 로드하여 각 조인트의 제한 정보 확인

    // 모델의 자유도 수 확인

    auto start_time = std::chrono::high_resolution_clock::now();

    int num_iterations_per_thread = num_iterations / thread_count;

    std::vector<std::thread> threads;
    for (unsigned int i = 0; i < thread_count; ++i)
    {
        threads.emplace_back(run_iteration, std::ref(model), end_effector_name, joint_limits_lower, joint_limits_upper, num_iterations_per_thread, i);
    }

    for (auto &thread : threads)
    {
        thread.join();
    }

    auto end_time = std::chrono::high_resolution_clock::now();

    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    double elapsed_time_sec = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();

    if (elapsed_time > 1000)
        std::cout << num_iterations << " forward kinematics calculation : " << elapsed_time_sec << " s" << std::endl;
    else
        std::cout << num_iterations << " forward kinematics calculation : " << elapsed_time << " ms" << std::endl;

    return 0;
}
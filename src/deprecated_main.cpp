#include <iostream>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <Eigen/Dense>
#include <random>
#include <chrono>

#ifndef URDF_DIR
#define URDF_DIR ""
#endif

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main()
{
    // 모델 생성
    Model *model = new Model();
    model->gravity = Vector3d(0., 0., -9.81);

    std::string urdf2_file = std::string(URDF_DIR) + "/panda.urdf";

    // URDF 파일로부터 모델 로드
    if (!Addons::URDFReadFromFile(urdf2_file.c_str(), model, false))
    {
        std::cerr << "URDF 파일을 로드하는 데 실패했습니다." << std::endl;
        return -1;
    }

    // 모델의 자유도 수 확인
    unsigned int dof = model->dof_count;
    std::cout << "모델의 자유도 수: " << dof << std::endl;

    // Panda 로봇의 관절 한계 (라디안)
    VectorNd joint_limits_lower(dof);
    VectorNd joint_limits_upper(dof);

    joint_limits_lower << -2.8973, -1.7628, -2.8973, -3.0718,
        -2.8973, -0.0175, -2.8973;
    joint_limits_upper << 2.8973, 1.7628, 2.8973, -0.0698,
        2.8973, 3.7525, 2.8973;

    // 난수 생성기 초기화
    std::random_device rd;
    std::mt19937 gen(rd());

    // 각 관절마다 난수 분포 설정
    std::vector<std::uniform_real_distribution<double>> distributions;
    for (unsigned int i = 0; i < dof; ++i)
    {
        distributions.emplace_back(joint_limits_lower[i], joint_limits_upper[i]);
    }

    // 엔드 이펙터의 링크 이름 설정
    const char *end_effector_name = "panda_hand"; // URDF 파일에서 엔드 이펙터 링크 이름 확인

    // 엔드 이펙터의 바디 ID 가져오기
    unsigned int end_effector_id = model->GetBodyId(end_effector_name);
    if (end_effector_id == std::numeric_limits<unsigned int>::max())
    {
        std::cerr << "엔드 이펙터 링크를 찾을 수 없습니다: " << end_effector_name << std::endl;
        delete model;
        return -1;
    }

    // 엔드 이펙터의 로컬 좌표 (엔드 이펙터 프레임의 원점)
    Vector3d end_effector_point = Vector3d::Zero();

    // 반복 횟수 설정
    const unsigned int num_iterations = 10000000;

    // 소요 시간 측정을 위한 시작 시간 기록
    auto start_time = std::chrono::high_resolution_clock::now();

    // 엔드 이펙터 위치를 저장할 변수 (마지막 계산 결과)
    Vector3d end_effector_position;

    // 10,000번 반복하여 엔드 이펙터 위치 계산
       for (unsigned int iter = 0; iter < num_iterations; ++iter) {
        // 임의의 관절 각도 생성
        VectorNd Q(dof);
        for (unsigned int i = 0; i < dof; ++i) {
            Q[i] = distributions[i](gen);
        }

        // 엔드 이펙터의 베이스 좌표계에서의 위치 계산
        end_effector_position = CalcBodyToBaseCoordinates(
            *model, Q, end_effector_id, end_effector_point, true);
    }

    // 소요 시간 측정을 위한 종료 시간 기록
    auto end_time = std::chrono::high_resolution_clock::now();

    // 총 소요 시간 계산 (밀리초 단위)
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time).count();

    // 결과 출력
    std::cout << num_iterations << "번 계산 소요 시간: " << elapsed_time << "밀리초" << std::endl;
    std::cout << "마지막 계산된 엔드 이펙터 위치: [" << end_effector_position.transpose() << "]" << std::endl;

    // 메모리 해제
    delete model;

    return 0;
}
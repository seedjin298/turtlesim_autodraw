#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>

#include "turtlesim_autodraw/circle.h"

ros::Publisher publisher;

void draw_circle(float speed, float radius, geometry_msgs::Twist &vel_msg, ros::Publisher &pub)
{
	// vel_msg에 포함된 3차원 속도 값 초기화
	vel_msg.linear.x = speed; //turtle은 전진만 하므로 x를 제외한 나머지 속도를 0으로 맞춘다.
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = speed/radius;

	// 출발 시각
	//double t0 = ros::Time::now().toSec();
	// 이동 거리
	//double distance_travelled = 0;

	while (ros::ok()) // 이동 거리가 아직 변 길이에 미치지 못했을 경우
	{
		// 속도 메시지 전송(전진 명령)
		pub.publish(vel_msg);
		// 짧은 이동 시간 측정하여 이동 거리(=속도x시간) 계산
		//double t1 = ros::Time::now().toSec();
		//distance_travelled = speed * (t1 - t0);
	}
	// 전진 속도 0으로 변경; 정지 명령
	vel_msg.linear.x = 0;	
	vel_msg.linear.z = 0;
	pub.publish(vel_msg);
}

bool handlecircle(turtlesim_autodraw::circle::Request &req, turtlesim_autodraw::circle::Response &res)
{

	ROS_INFO("[draw_circle_server] I just got incoming request!");
	
	// vel_msg 메시지 객체 생성
	geometry_msgs::Twist vel_msg;

	// service client로부터 요청된 변 길이 받아오기
	float speed = req.speed;
	ROS_INFO_STREAM("[draw_circle_server] req.speed=" << req.speed);

	// service client로부터 요청된 반복 회수 받아오기
	float radius = req.radius;
	ROS_INFO_STREAM("[draw_circle_server] req.radius=" << req.radius);

	while (ros::ok())
    {
		draw_circle(speed, radius, vel_msg, publisher);
    }

	// 여기서 return값에 의미는 없으나 콜백함수 형식을 바꿀 수 없으므로 return 값을 임의로 true로 설정
	return true;
}

int main(int argc, char **argv)
{
	// 노드/노드핸들 초기화
	ros::init(argc, argv, "draw_circle_server");
	ros::NodeHandle nh;
	ROS_INFO("[draw_circle_server] Startup");

	// 노드 반복 실행(spin) 주기 설정(ms)
	ros::Rate loop_rate(10);

	// 퍼블리셔 실행 (turtlesim 조종용 토픽 퍼블리시)
	publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	ROS_INFO("[draw_circle_server] Advertising publisher /turtle1/cmd_vel");

	// 서비스 서버 실행 (그릴 사각형 크기와 반복회수 받는 서비스 서버)
	ros::ServiceServer s = nh.advertiseService<turtlesim_autodraw::circle::Request, turtlesim_autodraw::circle::Response>("draw_circle", handlecircle);
	ROS_INFO("[draw_circle_server] Advertising service server draw_circle");

	// CTRL+C 등 종료 명령 받기 위해 ros::spin()대신 ros::ok(), 루프, ros::spinOnce() 조합
	while(ros::ok()) ros::spinOnce();

	return 0;
}
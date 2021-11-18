#include <iostream>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

using namespace std;

int main() {
  system("sudo kill -9 `sudo lsof -t -i:2002`");
  system("sudo kill -9 `sudo lsof -t -i:2003`");

  system("source /root/catkin_ws/devel/setup.bash");
  system("roslaunch pf_driver r2000.launch");

  sleep(5);
  system("source /home/idea/catkinTestGoogle/devel/setup.bash");
  system("roslaunch cartographer_ros demoR2000.launch");

  return 0;
}
#include <moveit_benchmark_suite/utils.h>

#include <sys/select.h>

#include <ros/ros.h>

void moveit_benchmark_suite::waitForKeyPress()
{
  fd_set rfds;
  struct timeval tv;
  int c;

  FD_ZERO(&rfds);
  FD_SET(0, &rfds);

  while (!ros::isShuttingDown())
  {
    // causes select() to return immidiatly
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    select(1, &rfds, NULL, NULL, &tv);
    if (FD_ISSET(0, &rfds))
    {
      // consume input
      while ((c = getchar()) != EOF && c != '\n')
        ;
      break;
    }

    FD_SET(0, &rfds);  // place stdin back in the fd set

    // use ros sleep instead of the timeout in select() because
    // it resumes quicker when a SIGINT is sent
    ros::Duration(0.1).sleep();
  }

  // close now
  if (ros::isShuttingDown())
    std::abort();
}

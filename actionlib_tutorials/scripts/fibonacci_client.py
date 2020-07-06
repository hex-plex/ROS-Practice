import rospy
from __future__ import print_function

import actionlib

import actionlib_tutorials.msg

def fibonacci_client():

    client = actionlib.SimpleActionClient('fibonacci',actionlib_tutorials.msg.FibonacciAction)
    client.wait_for_server()
    goal  =actionlib_tutorials.msg.FibonacciGoal(order=20)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == "__main__":
    try:
        rospy.init_node('fibonacci_client_py')
        result = fibonacci_client()
        print("Result:",', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion",file = sys.stderr)

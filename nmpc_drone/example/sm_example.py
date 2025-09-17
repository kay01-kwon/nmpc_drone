#! /usr/bin/env python3.8

import rospy
import smach
import smach_ros

class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, user_data):
        rospy.loginfo("Executing state Foo")
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'

class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, user_data):
        rospy.loginfo("Executing state Bar")
        return 'outcome1'

class Bas(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3'])

    def execute(self, user_data):
        rospy.loginfo("Executing state Bas")
        return 'outcome3'

def main():
    rospy.init_node('smach_example_sm')

    sm_top = smach.StateMachine(outcomes=['outcome5'])

    with sm_top:

        smach.StateMachine.add('BAS', Bas(),
                               transitions={'outcome3': 'SUB'})

        sm_sub = smach.StateMachine(outcomes=['outcome4'])

        with sm_sub:

            smach.StateMachine.add('FOO', Foo(),
                                   transitions={'outcome1':'BAR',
                                                'outcome2':'outcome4'})
            smach.StateMachine.add('BAR', Bar(),
                                   transitions={'outcome1': 'FOO'})

        smach.StateMachine.add('SUB', sm_sub,
                               transitions={'outcome4':'outcome5'})
    outcome = sm_top.execute()

if __name__ == '__main__':
    main()
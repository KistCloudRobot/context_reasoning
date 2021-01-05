:- module(predicate__service_globalVariable,
    []).
    
:- nb_setval(predicate__batteryState_low_robot_battery_threshold, 30).			%batteryState
:- nb_setval(predicate__batteryState_high_robot_battery_threshold, 70).			%batteryState

:- nb_setval(predicate__tilted_Object_low_angle_threshold, 10).					%tilted_Object
:- nb_setval(predicate__tilted_Object_high_angle_threshold, 350).				%tilted_Object

:- nb_setval(predicate__standing_Object_low_angle_threshold, 10).				%standing_Object
:- nb_setval(predicate__standing_Object_high_angle_threshold, 350).				%standing_Object


:- nb_setval(predicate__empty_hand_intersectPer_threshold, 0.01).				%empty_hand % 0.02 for vrep
:- nb_setval(predicate__overlap_hand_intersectPer_threshold, 0.01).				%overlap_hand %  0.07 for vrep

:- nb_setval(predicate__joint_opened_hand_finger_number, 2).					%opened_hand
:- nb_setval(predicate__joint_closed_hand_finger_number, 2).					%closed_hand
:- nb_setval(predicate__joint_check_joint_angle_threshold, 28.6).				%joint_check %10 for gazebo 15 for vrep

:- nb_setval(predicate__detected_object_min_time_threshold, 30).				%detected_object
:- nb_setval(predicate__detected_object_max_time_threshold, 60).				%detected_object


:- module(predicate,
    [
    objectSize/2,
    handSize/2,
    spaceSurroundedByHandSize/2,
    currentArtifactPerception/2,
    currentObjectPerception/2,
    currentArtifactVertex/2,
    currentObjectVertex/2,
    semanticObjectVertex/2,
    counter/2,
    set/2,
    vertex/3,
    degreeToRadian/2,
    currentJointPerception/2,
	semanticMapPerception/2,
    currentArtifactPose/2,
    currentObjectPose/2,
    currentHandPose/2,
    currentSpaceSurroundedByHandPose/2,
    currentSpaceSurroundedByHandVertex/2,
    currentRobotBodyPose/2,
    tilted_Object/1,
    standing_Object/1,
    intersects/3,
    intersects2/3,
    empty_hand/1,
    overlap_hand/2,
    closed_hand/1,
    opened_hand/1,
    graspedBy/2,
	close_empty_hand/1,
	close_overlap_hand/1,
    currentJointAngle/2,
    currentJointEffort/2,
    currentJointVelocity/2,
    currentHandPerception/2,
	currentRobotBodyPerception/2,
    type/2,
    detected_object/1,
	latest_detection_of_robot/2,
	latest_detection_of_object/2,
	latest_detection_of_semantic/2,
        latest_detection_of_object_test/3,
	currentHandVertex/2,
	semanticPose/2,
	batteryState/1,
	semanticObjectPose/2,
    joint_ex/4,
    joint_check/2,
    latest_detection_of_joint/2,
    empty_hand_test/3,
    closed_hand_test/2,
    detected_object_test/2,
    locatedAt/2,
    pre_currentObjectPerception/2
    ]).

set([], []).
set([H|T], [H|T1]) :- subtract(T, [H], T2), set(T2, T1).


counter([],0).
counter([_|T],N) :-
      counter(T,M),
      N is M+1 .

batteryState(State):-
	nb_getval(predicate__batteryState_low_robot_battery_threshold, Low_robot_battery_threshold),
  nb_getval(predicate__batteryState_high_robot_battery_threshold, High_robot_battery_threshold),

  get_timepoint(T),
	rdf(Robot, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
  	latest_battery_detection_of_instance(Robot,VPR),
	rdf(VPR, 'http://knowrob.org/kb/knowrob.owl#eventHasValue',literal(type(_,Rb))),atom_to_term(Rb,RB,_),
	(RB=<Low_robot_battery_threshold,State ='Low'),!;
	(RB>Low_robot_battery_threshold,RB=<High_robot_battery_threshold,State='Middle'),!;
	(RB>High_robot_battery_threshold,State='High').



pre_currentObjectPerception(Object, CurrentPerception):-
   rdf(Object, rdf:type, knowrob:'Artifact'),
   not(rdf(Object,  rdf:type, knowrob:'Hand')),
   latest_detection_of_object(Object, CurrentPerception).
   

currentArtifactPerception(Object, CurrentPerception):-
   rdfs_individual_of(Object,  knowrob:'Artifact'),
   latest_detection_of_artifact(Object, CurrentPerception).

currentObjectPerception(Object, CurrentPerception):-
   rdfs_individual_of(Object,  knowrob:'Artifact'),
   %not(rdf(Object,  rdf:type, knowrob:'Hand')),
   not(rdfs_individual_of(Object,  knowrob:'Hand')),
   latest_detection_of_object(Object, CurrentPerception).

currentHandPerception(Hand, CurrentPerception):-
   rdfs_individual_of(Hand,  knowrob:'Hand'),
   latest_detection_of_hand(Hand, CurrentPerception).

currentRobotBodyPerception(Hand, CurrentPerception):-
   rdfs_individual_of(Hand,  knowrob:'Robot'),
   latest_detection_of_robot(Hand, CurrentPerception).
   
currentJointPerception(Joint, CurrentPerception) :-
   rdfs_individual_of(Joint, 'http://knowrob.org/kb/srdl2-comp.owl#FixedUrdfJoint'),
   latest_detection_of_joint(Joint, CurrentPerception).

currentTouchPerception(TouchSensor, CurrentPerception) :-
	rdfs_individual_of(TouchSensor, 'http://knowrob.org/kb/srdl2-comp.owl#PressureSensor'),
	latest_detection_of_touch(TouchSensor, CurrentPerception).

semanticMapPerception(Object, Perception):-
   %rdfs_individual_of(Object, knowrob:'Artifact'),
   %rdf(Perception, rdf:type, knowrob:'SemanticMapPerception'),
   %rdf_has(Perception, knowrob:objectActedOn, Object).
   rdfs_individual_of(Object,  knowrob:'Artifact'),
   latest_detection_of_semantic(Object, Perception).
	
artifactSize(Object, Size):-
   rdfs_individual_of(Object,  knowrob:'Artifact'),
   rdf(Object, knowrob:depthOfObject, literal(type(_,S1x))),atom_to_term(S1x,S1X,_),
   rdf(Object, knowrob:widthOfObject, literal(type(_,S1y))),atom_to_term(S1y,S1Y,_),
   rdf(Object, knowrob:heightOfObject, literal(type(_,S1z))),atom_to_term(S1z,S1Z,_),
   Size = [S1X, S1Y, S1Z].

objectSize(Object, Size):-
   rdfs_individual_of(Object,  knowrob:'Artifact'),
   not(rdfs_individual_of(Object,  knowrob:'Hand')),
   %not(rdf(Object, rdf:type,  knowrob:'Hand')),
   rdf(Object, knowrob:depthOfObject, literal(type(_,S1x))),atom_to_term(S1x,S1X,_),
   rdf(Object, knowrob:widthOfObject, literal(type(_,S1y))),atom_to_term(S1y,S1Y,_),
   rdf(Object, knowrob:heightOfObject, literal(type(_,S1z))),atom_to_term(S1z,S1Z,_),
   Size = [S1X, S1Y, S1Z].

locatedAt(Object, Location) :-
   currentObjectPerception(Object, CurrentPerception),
   rdf(CurrentPerception, knowrob:eventOccursAt, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m02',literal(type(_,O1a))),atom_to_term(O1a,O1A,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m12',literal(type(_,O1b))),atom_to_term(O1b,O1B,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m22',literal(type(_,O1c))),atom_to_term(O1c,O1C,_),
   Location = [P1X, P1Y, P1Z, O1A, O1B, O1C].
   
handSize(Hand, HandSize):-
    rdfs_individual_of(Hand,  knowrob:'Hand'),
    rdf(Hand, knowrob:depthOfObject, literal(type(_,S1x))),atom_to_term(S1x,S1X,_),
    rdf(Hand, knowrob:widthOfObject, literal(type(_,S1y))),atom_to_term(S1y,S1Y,_),
    rdf(Hand, knowrob:heightOfObject, literal(type(_,S1z))),atom_to_term(S1z,S1Z,_),
    HandSize = [S1X, S1Y, S1Z].

spaceSurroundedByHandSize(Space, SpaceSize):-
    rdfs_individual_of(Space, arbi:'SpaceSurroundedByHand'),
    rdf(Space, knowrob:depthOfObject, literal(type(_,S1x))),atom_to_term(S1x,S1X,_),
    rdf(Space, knowrob:widthOfObject, literal(type(_,S1y))),atom_to_term(S1y,S1Y,_),
    rdf(Space, knowrob:heightOfObject, literal(type(_,S1z))),atom_to_term(S1z,S1Z,_),
    SpaceSize = [S1X, S1Y, S1Z].

currentTipContactPerception(TipList) :-
      (rdfs_individual_of(Subject,  knowrob:'UrdfLink'),
      rdfs_individual_of(Subject,  knowrob:'Finger')),
      append([], Subject, TipList).


currentHandTorque(Hand, Torques) :-
      rdfs_individual_of(Hand,  knowrob:'Hand'),
      findall(Torque, (rdfs_individual_of(Fingertip, knowrob:'FingerTip'), currentFingerTipTorque(Fingertip,Torque)),Torques).
      
currentHandForce(Hand, Forces) :-
      rdfs_individual_of(Hand,  knowrob:'Hand'),
      findall(Force, (rdfs_individual_of(Fingertip, knowrob:'FingerTip'), currentFingerTipForce(Fingertip,Force)),Forces).

currentArtifactPose(Object, Pose) :-
   currentArtifactPerception(Object, CurrentPerception),
   rdf(CurrentPerception, knowrob:eventOccursAt, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m02',literal(type(_,O1a))),atom_to_term(O1a,O1A,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m12',literal(type(_,O1b))),atom_to_term(O1b,O1B,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m22',literal(type(_,O1c))),atom_to_term(O1c,O1C,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m32',literal(type(_,O1d))),atom_to_term(O1d,O1D,_),

   Pose = [P1X, P1Y, P1Z, O1A, O1B, O1C, O1D].

currentObjectPose(Object, Pose) :-
   currentObjectPerception(Object, CurrentPerception),
   rdf(CurrentPerception, knowrob:eventOccursAt, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m02',literal(type(_,O1a))),atom_to_term(O1a,O1A,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m12',literal(type(_,O1b))),atom_to_term(O1b,O1B,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m22',literal(type(_,O1c))),atom_to_term(O1c,O1C,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m32',literal(type(_,O1d))),atom_to_term(O1d,O1D,_),
   Pose = [P1X, P1Y, P1Z, O1A, O1B, O1C, O1D].

%dont use
currentVisibleObjectPose(Object, Pose) :-
   currentObjectPerception(Object, CurrentPerception),
   rdf(CurrentPerception, knowrob:eventOccursAt, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m02',literal(type(_,O1a))),atom_to_term(O1a,O1A,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m12',literal(type(_,O1b))),atom_to_term(O1b,O1B,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m22',literal(type(_,O1c))),atom_to_term(O1c,O1C,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m32',literal(type(_,O1d))),atom_to_term(O1d,O1D,_),

   Pose = [P1X, P1Y, P1Z, O1A, O1B, O1C, O1D].

currentHandPose(Hand, Pose) :-
   currentHandPerception(Hand, CurrentPerception),
   rdf(CurrentPerception, knowrob:eventOccursAt, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m02',literal(type(_,O1a))),atom_to_term(O1a,O1A,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m12',literal(type(_,O1b))),atom_to_term(O1b,O1B,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m22',literal(type(_,O1c))),atom_to_term(O1c,O1C,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m32',literal(type(_,O1d))),atom_to_term(O1d,O1D,_),
   Pose = [P1X, P1Y, P1Z, O1A, O1B, O1C, O1D].

currentSpaceSurroundedByHandPose(Space, Pose) :-
   currentHandPerception(Hand, CurrentPerception),
   rdf(CurrentPerception, knowrob:eventOccursAt, Space_Matrix),
   rdfs_individual_of(Space,  arbi:'SpaceSurroundedByHand'),
   %rdf(Space_Matrix,knowrob:'inFrontOf-Generally',Object_Matrix),
   rdf(Space_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Space_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Space_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   Pose = [P1X, P1Y, P1Z].

currentRobotBodyPose(Body, Pose) :-
   currentRobotBodyPerception(Body, CurrentPerception),
   rdf(CurrentPerception, knowrob:eventOccursAt, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m02',literal(type(_,O1a))),atom_to_term(O1a,O1A,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m12',literal(type(_,O1b))),atom_to_term(O1b,O1B,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m22',literal(type(_,O1c))),atom_to_term(O1c,O1C,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m32',literal(type(_,O1d))),atom_to_term(O1d,O1D,_),
   Pose = [P1X, P1Y, P1Z, O1A, O1B, O1C, O1D].
   

semanticPose(Object, Pose) :-
   semanticPerception(Object, SemanticPerception),
   rdf(SemanticPerception, knowrob:eventOccursAt, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m02',literal(type(_,O1a))),atom_to_term(O1a,O1A,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m12',literal(type(_,O1b))),atom_to_term(O1b,O1B,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m22',literal(type(_,O1c))),atom_to_term(O1c,O1C,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m32',literal(type(_,O1d))),atom_to_term(O1d,O1D,_),
   Pose = [P1X, P1Y, P1Z, O1A, O1B, O1C, O1D].

semanticObjectPose(Object, Pose):-
   semanticMapPerception(Object, SemanticMapPerception),
   rdf(SemanticMapPerception, knowrob:eventOccursAt, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m02',literal(type(_,O1a))),atom_to_term(O1a,O1A,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m12',literal(type(_,O1b))),atom_to_term(O1b,O1B,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m22',literal(type(_,O1c))),atom_to_term(O1c,O1C,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m32',literal(type(_,O1d))),atom_to_term(O1d,O1D,_),
   Pose = [P1X, P1Y, P1Z, O1A, O1B, O1C, O1D].
	
currentJointAngle(Joint, Angle):-
   currentJointPerception(Joint, CurrentPerception),
   rdf(CurrentPerception,knowrob:'radius',literal(type(_,J1r))),atom_to_term(J1r,J1R,_),
   Angle = J1R.
   
currentJointVelocity(Joint, Velocity):-
   currentJointPerception(Joint, CurrentPerception),
   rdf(CurrentPerception,knowrob:'velocity',literal(type(_,J1v))),atom_to_term(J1v,J1V,_),
   Velocity = J1V.

currentJointEffort(Joint, Effort):-
   currentJointPerception(Joint, CurrentPerception),
   rdf(CurrentPerception,knowrob:'effort',literal(type(_,J1e))),atom_to_term(J1e,J1E,_),
   Effort = J1E.

tilted_Object(Object):-
   nb_getval(predicate__tilted_Object_low_angle_threshold, Low_angle_threshold),
   nb_getval(predicate__tilted_Object_high_angle_threshold, High_angle_threshold),
   currentObjectPose(Object, Pose),
   nth0(3, Pose, Tilted_RadiusX),
   nth0(4, Pose, Tilted_RadiusY),
   once(
   (Tilted_RadiusX < 0 , RadiusX is Tilted_RadiusX + 360);
   (Tilted_RadiusX >= 0 , RadiusX is Tilted_RadiusX )
   ),
   once(
   (Tilted_RadiusY < 0 , RadiusY is Tilted_RadiusY + 360);
   (Tilted_RadiusY >= 0 , RadiusY is Tilted_RadiusY )
   ),
   (RadiusX>Low_angle_threshold, RadiusX<High_angle_threshold),
   (RadiusY>Low_angle_threshold, RadiusY<High_angle_threshold).

standing_Object(Object):-
   nb_getval(predicate__standing_Object_low_angle_threshold, Low_angle_threshold),
   nb_getval(predicate__standing_Object_high_angle_threshold, High_angle_threshold),
   currentObjectPose(Object, Pose),
   nth0(3, Pose, Tilted_RadiusX),
   nth0(4, Pose, Tilted_RadiusY),
   once(
   (Tilted_RadiusX < 0 , RadiusX is Tilted_RadiusX + 360);
   (Tilted_RadiusX >= 0 , RadiusX is Tilted_RadiusX )
   ),
   once(
   (Tilted_RadiusY < 0 , RadiusY is Tilted_RadiusY + 360);
   (Tilted_RadiusY >= 0 , RadiusY is Tilted_RadiusY )
   ),
   (RadiusX=<Low_angle_threshold; RadiusX>=High_angle_threshold),
   (RadiusY=<Low_angle_threshold; RadiusY>=High_angle_threshold).

%추가 및 수정
currentArtifactVertex(Object, Vertice):-
   currentArtifactPose(Object, ObjectPose),
   artifactSize(Object, ObjectSize),
   vertex(ObjectPose, ObjectSize, Vertice).

%추가 및 수정
currentObjectVertex(Object, Vertice):-
   currentObjectPose(Object, ObjectPose),
   objectSize(Object, ObjectSize),
   vertex(ObjectPose, ObjectSize, Vertice).

%추가 및 수정
semanticObjectVertex(Object, Vertice):-
   semanticObjectPose(Object, ObjectPose),
   objectSize(Object, ObjectSize),
   vertex(ObjectPose, ObjectSize, Vertice).

%추가 및 수정
currentHandVertex(Hand, Vertice):-
   currentHandPose(Hand, HandPose),
   handSize(Hand, HandSize),
   vertex(HandPose, HandSize, Vertice).

%추가 및 수정
currentSpaceSurroundedByHandVertex(Space, Vertice):-
   currentSpaceSurroundedByHandPose(Space, SpacePose),
   spaceSurroundedByHandSize(Space, SpaceSize),
   vertex(SpacePose, SpaceSize, Vertice).

orientation(Orientation, Matrix):-
   nth0(0, Orientation, OriA),
   nth0(1, Orientation, OriB),
   nth0(2, Orientation, OriC),

   %degreeToRadian(OriA,OriAR),
   %degreeToRadian(OriB,OriBR),
   %degreeToRadian(OriC,OriCR),
   
   SinA is sin(OriA),
   CosA is cos(OriA),
   SinB is sin(OriB),
   CosB is cos(OriB),
   SinC is sin(OriC),
   CosC is cos(OriC),
   
   
   Rx = [[1,0,0],[0,CosA,-SinA],[0,SinA,CosA]],
   Ry = [[CosB,0,SinB],[0,1,0],[-SinB,0,CosB]],
   Rz = [[CosC,-SinC,0],[SinC,CosC,0],[0,0,1]],
   
   %Rx = [[1,0,0],[0,CosC,-SinC],[0,SinC,CosC]],
   %Ry = [[CosB,0,SinB],[0,1,0],[-SinB,0,CosB]],
   %Rz = [[CosA,-SinA,0],[SinA,CosA,0],[0,0,1]],
   
   
   matrix_multiply(Ry,Rx,Ryx),
   matrix_multiply(Rz,Ryx,Rzyx),
   
   Matrix = Rzyx.

vertex(Pose, Size, Vertice):-
   nth0(0, Pose, PoseXO),
   nth0(1, Pose, PoseYO),
   nth0(2, Pose, PoseZO),

   PoseXYZ=[[PoseXO],[PoseYO],[PoseZO]],
   PoseX = 0, PoseY = 0, PoseZ = 0,

   nth0(3, Pose, OriA),
   nth0(4, Pose, OriB),
   nth0(5, Pose, OriC),


   orientation([OriA,OriB,OriC],Ozyx),

   nth0(0, Size, SizeX),
   nth0(1, Size, SizeY),
   nth0(2, Size, SizeZ),

   P1X is PoseX - (SizeX/2),
   P1Y is PoseY - (SizeY/2),
   P1Z is PoseZ - (SizeZ/2),
   matrix_multiply(Ozyx,[[P1X], [P1Y], [P1Z]],P1R),%flatten(P1R,P1), %P1 = [P1X, P1Y, P1Z],
   P2X is PoseX - (SizeX/2),
   P2Y is PoseY + (SizeY/2),
   P2Z is PoseZ - (SizeZ/2),
   matrix_multiply(Ozyx,[[P2X], [P2Y], [P2Z]],P2R),%flatten(P2R,P2), %P2 = [P2X, P2Y, P2Z],
   P3X is PoseX + (SizeX/2),
   P3Y is PoseY + (SizeY/2),
   P3Z is PoseZ - (SizeZ/2),
   matrix_multiply(Ozyx,[[P3X], [P3Y], [P3Z]],P3R),%flatten(P3R,P3), %P3 = [P3X, P3Y, P3Z],
   P4X is PoseX + (SizeX/2),
   P4Y is PoseY - (SizeY/2),
   P4Z is PoseZ - (SizeZ/2),
   matrix_multiply(Ozyx,[[P4X], [P4Y], [P4Z]],P4R),%flatten(P4R,P4), %P4 = [P4X, P4Y, P4Z],
   P5X is PoseX - (SizeX/2),
   P5Y is PoseY - (SizeY/2),
   P5Z is PoseZ + (SizeZ/2),
   matrix_multiply(Ozyx,[[P5X], [P5Y], [P5Z]],P5R),%flatten(P5R,P5), %P5 = [P5X, P5Y, P5Z],
   P6X is PoseX - (SizeX/2),
   P6Y is PoseY + (SizeY/2),
   P6Z is PoseZ + (SizeZ/2),
   matrix_multiply(Ozyx,[[P6X], [P6Y], [P6Z]],P6R),%flatten(P6R,P6), %P6 = [P6X, P6Y, P6Z],
   P7X is PoseX + (SizeX/2),
   P7Y is PoseY + (SizeY/2),
   P7Z is PoseZ + (SizeZ/2),
   matrix_multiply(Ozyx,[[P7X], [P7Y], [P7Z]],P7R),%flatten(P7R,P7), %P7 = [P7X, P7Y, P7Z],
   P8X is PoseX + (SizeX/2),
   P8Y is PoseY - (SizeY/2),
   P8Z is PoseZ + (SizeZ/2),
   matrix_multiply(Ozyx,[[P8X], [P8Y], [P8Z]],P8R),%flatten(P8R,P8), %P8 = [P8X, P8Y, P8Z],

   matrix_sum(P1R,PoseXYZ,P1F),transpose(P1F,P1T),flatten(P1T,P1),
   matrix_sum(P2R,PoseXYZ,P2F),transpose(P2F,P2T),flatten(P2T,P2),
   matrix_sum(P3R,PoseXYZ,P3F),transpose(P3F,P3T),flatten(P3T,P3),
   matrix_sum(P4R,PoseXYZ,P4F),transpose(P4F,P4T),flatten(P4T,P4),
   matrix_sum(P5R,PoseXYZ,P5F),transpose(P5F,P5T),flatten(P5T,P5),
   matrix_sum(P6R,PoseXYZ,P6F),transpose(P6F,P6T),flatten(P6T,P6),
   matrix_sum(P7R,PoseXYZ,P7F),transpose(P7F,P7T),flatten(P7T,P7),
   matrix_sum(P8R,PoseXYZ,P8F),transpose(P8F,P8T),flatten(P8T,P8),
   Vertice = [P1, P2, P3, P4, P5, P6, P7, P8].

%추가 및 수정
intersects(Hand, Object, IntersectPer):-
   rdfs_individual_of(Hand,  knowrob:'Hand'),
   rdfs_individual_of(Object,  knowrob:'Artifact'),
 
   currentObjectVertex(Object, ObjectPoint),
   currentHandVertex(Hand, SpacePoint),
   
   %currentSpaceSurroundedByHandVertex(Space, SpacePoint),
   
   nth0(0, SpacePoint, Space1), nth0(0, Space1, X1E), nth0(1, Space1, Y1E), nth0(2, Space1, Z1E),
   nth0(6, SpacePoint, Space2), nth0(0, Space2, X2E), nth0(1, Space2, Y2E), nth0(2, Space2, Z2E),
   nth0(0, ObjectPoint, Object3), nth0(0, Object3, X3E), nth0(1, Object3, Y3E), nth0(2, Object3, Z3E),
   nth0(6, ObjectPoint, Object4), nth0(0, Object4, X4E), nth0(1, Object4, Y4E), nth0(2, Object4, Z4E),

   (X1E > X2E -> X1 = X2E, X2 = X1E;X1 = X1E, X2 = X2E),
   (Y1E > Y2E -> Y1 = Y2E, Y2 = Y1E;Y1 = Y1E, Y2 = Y2E),
   (Z1E > Z2E -> Z1 = Z2E, Z2 = Z1E;Z1 = Z1E, Z2 = Z2E),
   (X3E > X4E -> X3 = X4E, X4 = X3E;X3 = X3E, X4 = X4E),
   (Y3E > Y4E -> Y3 = Y4E, Y4 = Y3E;Y3 = Y3E, Y4 = Y4E),
   (Z3E > Z4E -> Z3 = Z4E, Z4 = Z3E;Z3 = Z3E, Z4 = Z4E),
   
   once((X2<X3, LengthX is 0);
   (X1<X3, X3<X2, X2<X4, LengthX is X2-X3);
   (X1<X3, X3<X2, X4<X2, LengthX is X4-X3);
   (X3<X1, X2<X4, LengthX is X2-X1);
   (X3<X1, X1<X4, X4<X2, LengthX is X4-X1);
   (X4<X1, LengthX is 0)),
   
   once((Y2<Y3, LengthY is 0);
   (Y1<Y3, Y3<Y2, Y2<Y4, LengthY is Y2-Y3);
   (Y1<Y3, Y3<Y2, Y4<Y2, LengthY is Y4-Y3);
   (Y3<Y1, Y2<Y4, LengthY is Y2-Y1);
   (Y3<Y1, Y1<Y4, Y4<Y2, LengthY is Y4-Y1);
   (Y4<Y1, LengthY is 0)),
   
   once((Z2<Z3, LengthZ is 0);
   (Z1<Z3, Z3<Z2, Z2<Z4, LengthZ is Z2-Z3);
   (Z1<Z3, Z3<Z2, Z4<Z2, LengthZ is Z4-Z3);
   (Z3<Z1, Z2<Z4, LengthZ is Z2-Z1);
   (Z3<Z1, Z1<Z4, Z4<Z2, LengthZ is Z4-Z1);
   (Z4<Z1, LengthZ is 0)),
   
   SpaceLengthX is X2-X1, SpaceLengthY is Y2-Y1, SpaceLengthZ is Z2-Z1,
   SpaceVolume is SpaceLengthX*SpaceLengthY*SpaceLengthZ,
   IntersectVolume is LengthX*LengthY*LengthZ,
   IntersectPer is IntersectVolume/SpaceVolume.
   
%temp
intersects2(Space1Vertice, Space2Vertice, IntersectPer) :-
   nth0(0, Space1Vertice, S1V0), nth0(0, S1V0, X1), nth0(1, S1V0, Y1), nth0(2, S1V0, Z1),
   nth0(6, Space1Vertice, S1V6), nth0(0, S1V6, X2), nth0(1, S1V6, Y2), nth0(2, S1V6, Z2),
   nth0(0, Space2Vertice, S2V0), nth0(0, S2V0, X3), nth0(1, S2V0, Y3), nth0(2, S2V0, Z3),
   nth0(6, Space2Vertice, S2V6), nth0(0, S2V6, X4), nth0(1, S2V6, Y4), nth0(2, S2V6, Z4),
   
   once((X2<X3, LengthX is 0);
   (X1<X3, X3<X2, X2<X4, LengthX is X2-X3);
   (X1<X3, X3<X2, X4<X2, LengthX is X4-X3);
   (X3<X1, X2<X4, LengthX is X2-X1);
   (X3<X1, X1<X4, X4<X2, LengthX is X4-X1);
   (X4<X1, LengthX is 0)),
   
   once((Y2<Y3, LengthY is 0);
   (Y1<Y3, Y3<Y2, Y2<Y4, LengthY is Y2-Y3);
   (Y1<Y3, Y3<Y2, Y4<Y2, LengthY is Y4-Y3);
   (Y3<Y1, Y2<Y4, LengthY is Y2-Y1);
   (Y3<Y1, Y1<Y4, Y4<Y2, LengthY is Y4-Y1);
   (Y4<Y1, LengthY is 0)),
   
   once((Z2<Z3, LengthZ is 0);
   (Z1<Z3, Z3<Z2, Z2<Z4, LengthZ is Z2-Z3);
   (Z1<Z3, Z3<Z2, Z4<Z2, LengthZ is Z4-Z3);
   (Z3<Z1, Z2<Z4, LengthZ is Z2-Z1);
   (Z3<Z1, Z1<Z4, Z4<Z2, LengthZ is Z4-Z1);
   (Z4<Z1, LengthZ is 0)),
   
   SpaceLengthX is X2-X1, SpaceLengthY is Y2-Y1, SpaceLengthZ is Z2-Z1,
   SpaceVolume is SpaceLengthX*SpaceLengthY*SpaceLengthZ,
   IntersectVolume is LengthX*LengthY*LengthZ,
   IntersectPer is IntersectVolume/SpaceVolume.

empty_hand(Hand):-
   nb_getval(predicate__empty_hand_intersectPer_threshold, IntersectPer_threshold),
   rdfs_individual_of(Hand,  knowrob:'Hand'),
   findall(Object,  (currentObjectPerception(Object, CurrentPerception)), Objects),
   (counter(Objects,0)->true;
   foreach(member(O,Objects), intersects(Hand, O, IntersectPer)),
   IntersectPer =< IntersectPer_threshold).

empty_hand_test(Hand,Object,IntersectPer):-
   nb_getval(predicate__empty_hand_intersectPer_threshold, IntersectPer_threshold),
   rdfs_individual_of(Hand,  knowrob:'Hand'),
   intersects(Hand, Object, IntersectPer),
   findall(Object,  (currentObjectPerception(Object, CurrentPerception)), Objects),
   foreach(member(O,Objects), intersects(Hand, O, IntersectPer)).

type(Object, Type):-
	rdf(Object, rdf:type, Type).

%추가 및 수정
overlap_hand(Hand,Object):-
   nb_getval(predicate__overlap_hand_intersectPer_threshold, IntersectPer_threshold),
   intersects(Hand, Object, IntersectPer),
   IntersectPer > IntersectPer_threshold.
	
degreeToRadian(Degree, Radian):-
   (Degree =< 0 -> D is 360 + Degree, Radian is D*pi/180 ; Radian is Degree*pi/180).

radianToDegree(Radian,Degree):-
	Degree1 is round(Radian * 57.295),
	once((Degree1 < 0, Degree is 360 + Degree1);-
	(Degree1 >= 360, Degree is mod(Degree1,360));
	(Degree1 >= 0, Degree1 < 360, Degree is Degree1)).
	
opened_hand(Hand):-
   nb_getval(predicate__joint_opened_hand_finger_number, Finger_number),
   joint_check(Hand,Number),
   Number >= Finger_number.
   
closed_hand(Hand):-
   nb_getval(predicate__joint_closed_hand_finger_number, Finger_number),
   joint_check(Hand,Number),
   Number < Finger_number.

closed_hand_test(Hand, Number):-
   nb_getval(predicate__joint_closed_hand_finger_number, Finger_number),
   joint_check(Hand,Number).
   
joint_ex(Hand,A1,A2,A3):-
   rdfs_individual_of(Hand, knowrob:'Hand'),
   findall(Angle,  (rdf(Hand, 'http://knowrob.org/kb/srdl2-comp.owl#subComponent',  Joint), 	rdfs_individual_of(Joint, 'http://knowrob.org/kb/srdl2-comp.owl#FixedUrdfJoint'), currentJointAngle(Joint,Angle)),  Angles),
   [A1,A2,A3|_] = Angles.


% if you have a problem, use radianToDegree
joint_check(Hand,Number):-
   nb_getval(predicate__joint_check_joint_angle_threshold, Joint_threshold),
   rdfs_individual_of(Hand,  knowrob:'Hand'),
   %rdf(Hand, rdf:type,  knowrob:'Hand'),
   findall(Angle,  (rdf(Hand, 'http://knowrob.org/kb/srdl2-comp.owl#subComponent',  Joint), rdfs_individual_of(Joint, 'http://knowrob.org/kb/srdl2-comp.owl#FixedUrdfJoint'), currentJointAngle(Joint,Angle)),  Angles),
   [A1,A2,A3|_] = Angles,
   once((A1 < Joint_threshold, AN1 is 1);
	(AN1 is 0)),
   once((A2 < Joint_threshold, AN2 is 1);
	(AN2 is 0)),
   once((A3 < Joint_threshold, AN3 is 1);
	(AN3 is 0)),
   Number is (AN1+AN2+AN3).

%   once((A1 < Joint_threshold, AN1 is 1);

%	(AN1 is 0)),

%   once((A2 < Joint_threshold, AN2 is 1);

%	(AN2 is 0)),

%   once((A3 < Joint_threshold, AN3 is 1);

%	(AN3 is 0)),

%   Number is (AN1+AN2+AN3).


%추가 및 수정
close_empty_hand(Hand):-
	closed_hand(Hand),
	empty_hand(Hand).

%추가 및 수정
close_overlap_hand(Hand):-
	closed_hand(Hand),
	overlap_hand(Hand,Object).

graspedBy(Object, Hand):-
   closed_hand(Hand),
   overlap_hand(Hand,Object).
/*
graspedBy(Hand, Object):-
   closed_hand(Hand),
   (rdf(Perception,'graspedBy',Hand),rdf_has(Perception,knowrob:'startTime',StartTime1),rdf_split_(StratTime1,_,T1),
   currentObjectPerception(Object, Perception2),rdf_has(Perception2,knowrob:'startTime',StartTime2),iri_xml_namespace(StartTime2,T2),
   T2 - T1 < 4 -> true;
   
   overlap_hand(Hand, Object),
   
   currentObjectPerception(Object, Perception),
   rdf_retractall(S,'graspedBy',Hand),
   rdf_assert(Perception,'graspedBy',Hand)).
*/
detected_object(Object) :-
    nb_getval(predicate__detected_object_min_time_threshold, Min_time_threshold),
    nb_getval(predicate__detected_object_max_time_threshold, Max_time_threshold),
   
   % findall(O,
   %  (
   %  (\+rdf(O, rdf:type,  knowrob:'Hand')),
   %  (rdfs_individual_of(O,  knowrob:'Artifact');rdfs_individual_of(O,  knowrob:'HumanScaleObject'))
   %  ),
   %  Objects).
   % set(Objects,Objects2),
   % member(Object,Objects2),
   % latest_detection_of_object(Object, CurrentPerception),
   currentObjectPerception(Object, CurrentPerception),
   rdf_has(CurrentPerception, knowrob:'startTime', StartTimeR),

   rdf_split_url(_, StartTt, StartTimeR),
   atom_concat('timepoint_', StartTAtom, StartTt),
   atom_number(StartTAtom,IntTime),
   get_time(ST),
   A is ST-IntTime,
   (ST - IntTime =< Min_time_threshold).%, ST - IntTime =< Max_time_threshold).
	

detected_object_test(Object, A) :-
    nb_getval(predicate__detected_object_min_time_threshold, Min_time_threshold),
    nb_getval(predicate__detected_object_max_time_threshold, Max_time_threshold),
   
   % findall(O,
   %  (
   %  (\+rdf(O, rdf:type,  knowrob:'Hand')),
   %  (rdfs_individual_of(O,  knowrob:'Artifact');rdfs_individual_of(O,  knowrob:'HumanScaleObject'))
   %  ),
   %  Objects).
   % set(Objects,Objects2),
   % member(Object,Objects2),
   % latest_detection_of_object(Object, CurrentPerception),
   currentObjectPerception(Object, CurrentPerception),
   rdf_has(CurrentPerception, knowrob:'startTime', StartTimeR),

   rdf_split_url(_, StartTt, StartTimeR),
   atom_concat('timepoint_', StartTAtom, StartTt),
   atom_number(StartTAtom,IntTime),
   get_time(ST),
   A is ST-IntTime,
   (ST - IntTime =< Min_time_threshold).%, ST - IntTime =< Max_time_threshold).

   
latest_detection_of_object_test(Object, LatestDetection, Latest) :-

  ((rdf_has(Object, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Object,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              (rdfs_individual_of(D_i,  knowrob:'VisualObjectPerception')),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).

latest_detection_of_artifact(Object, LatestDetection) :-

  ((rdf_has(Object, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Object,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              (rdfs_individual_of(D_i,  knowrob:'Perceiving')),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).

latest_detection_of_object(Object, LatestDetection) :-

  ((rdf_has(Object, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Object,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              (rdfs_individual_of(D_i,  knowrob:'VisualObjectPerception')),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).
    
latest_detection_of_semantic(Object, LatestDetection) :-

  ((rdf_has(Object, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Object,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              (rdfs_individual_of(D_i,  knowrob:'SemanticMapPerception')),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).

latest_detection_of_robot(Object, LatestDetection) :-

  ((rdf_has(Object, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Object,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              (rdfs_individual_of(D_i,  knowrob:'VisualRobotBodyPerception')),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).
	
latest_detection_of_touch(Subject, LatestDetection) :-

  ((rdf_has(Subject, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Subject,St], (rdf_has(D_i, knowrob:objectActedOn, Subject),
                              ( rdfs_individual_of(D_i,  knowrob:'TouchPerception')),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),

    % compute the homography for the newest perception
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).


latest_battery_detection_of_instance(Object, LatestDetection) :-

  ((rdf_has(Object, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Object,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              rdfs_individual_of(D_i,  arbi:'BatteryPerception'),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),

    % compute the homography for the newest perception
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).

latest_detection_of_joint(Object, LatestDetection) :-

  ((rdf_has(Object, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Object,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              (rdfs_individual_of(D_i,  knowrob:'JointPerception')),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),

    % compute the homography for the newest perception
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).
	
latest_detection_of_hand(Object, LatestDetection) :-

  ((rdf_has(Object, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Hand,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              (rdfs_individual_of(D_i,  knowrob:'VisualRobotHandPerception')),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),

    % compute the homography for the newest perception
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).




    % compatibility with Prolog < 5.8
:- if(\+current_predicate(atomic_list_concat, _)).

  atomic_list_concat(List, Atom) :-
    concat_atom(List, Atom).

  atomic_list_concat(List, Separator, Atom) :-
    concat_atom(List, Separator, Atom).

:- endif.
detection_starttime(Detection, StartTime) :-

  % start time is asserted
  rdf_triple(knowrob:startTime, Detection, StartTtG),
  rdf_split_url(_, StartTt, StartTtG),
  atom_concat('timepoint_', StartTAtom, StartTt),
  term_to_atom(StartTime, StartTAtom),! ;

  rdf_split_url(_, StartTt, Detection),
  atom_concat('timepoint_', StartTAtom, StartTt),
  term_to_atom(StartTime, StartTAtom).



detection_endtime(Detection, EndTime) :-

  rdf_triple(knowrob:endTime, Detection, EndTtG),
  rdf_split_url(_, EndTt, EndTtG),
  atom_concat('timepoint_', EndTAtom, EndTt),
  term_to_atom(EndTime, EndTAtom),!;

  ( rdf_has(LaterDetection, knowrob:previousDetectionOfObject, Detection),
    rdf_triple(knowrob:startTime, LaterDetection, EndTtG),
    rdf_split_url(_, EndTt, EndTtG),
    atom_concat('timepoint_', EndTAtom, EndTt),
    term_to_atom(EndTime, EndTAtom),! );

  ( rdf_has(Detection, knowrob:objectActedOn, Object),
    rdf_has(Destruction, knowrob:inputsDestroyed, Object),
    Destruction \= Detection,
    rdfs_individual_of(Destruction,  knowrob:'PhysicalDestructionEvent'),
    rdf_triple(knowrob:startTime, Detection, StT),
    rdf_triple(knowrob:startTime, Destruction, EndTtG),
    rdf_triple(knowrob:after, StT, EndTtG),
    rdf_split_url(_, EndTt, EndTtG),
    atom_concat('timepoint_', EndTAtom, EndTt),
    term_to_atom(EndTime, EndTAtom),! );

  % otherwise take the current time (plus a second to avoid glitches)
  ( get_time(ET), EndTime is ET + 1.0).

%% compare_object_detections(-Delta, +P1, +P2) is det.
%
% Sort detections by their start time
%
% @param Delta  One of '>', '<', '='
% @param P1     List [_, _, Time] as used in latest_detection_of_object, latest_detection_of_type, latest_inferred_object_set
% @param P2     List [_, _, Time] as used in latest_detection_of_object, latest_detection_of_type, latest_inferred_object_set
%
compare_object_detections(Delta, P1, P2) :-

    nth0(2, P1, St1),
    nth0(2, P2, St2),
    compare(Delta, St2, St1).

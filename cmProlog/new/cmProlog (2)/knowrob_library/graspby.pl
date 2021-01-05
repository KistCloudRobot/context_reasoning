
:- module(graspby,
    [
    objectSize/2,
    leftHandSize/2,
    rightHandSize/2,
    currentObjectPerception/2,
    currentTipContactPerception/1,
    currentContactPerception/1,
    currentFingerTipTouchPerception/2,
   currentHandTouchPerception/1,
   counter/2,
   findComponent/1,
   set/2,
    currentJointPerception/2,
    currentObjectPose/2,
    currentHandPose/2,
    currentJointAngle/2,
    currentJointTorque/2,
    tilted_Object/1,
    standing_Object/1,
    intersects/3,
    empty_Hand/1,
    full_Hand/2,
    not_intersects/3,
    closed_Hand/1,
    opened_Hand/1,
    graspedBy/1,
    graspedBy/2,
    currentJointEffort/2,
    check_Velocity/1,
    check_Effort/1,
    currentTouchPerception/2,
    currentHandPerception/2

    ]).

objectSize(Object, Size):-
   rdfs_individual_of(Object,  knowrob:'Artifact'),
   not(rdf(Object, rdf:type,  knowrob:'Hand')),
   rdf(Object, knowrob:depthOfObject, literal(type(_,S1x))),atom_to_term(S1x,S1X,_),
   rdf(Object, knowrob:widthOfObject, literal(type(_,S1y))),atom_to_term(S1y,S1Y,_),
   rdf(Object, knowrob:heightOfObject, literal(type(_,S1z))),atom_to_term(S1z,S1Z,_),
   Size = [S1X, S1Y, S1Z].

leftHandSize(Hand, HandSize):-
   rdf(Hand, rdf:type,  knowrob:'LeftHand'),
   rdf(Hand, knowrob:depthOfObject, literal(type(_,S1x))),atom_to_term(S1x,S1X,_),
   rdf(Hand, knowrob:widthOfObject, literal(type(_,S1y))),atom_to_term(S1y,S1Y,_),
   rdf(Hand, knowrob:heightOfObject, literal(type(_,S1z))),atom_to_term(S1z,S1Z,_),
   HandSize = [S1X, S1Y, S1Z].

rightHandSize(Hand, Size):-
   rdf(Hand, rdf:type,  knowrob:'RightHand'),
   rdf(Hand, knowrob:depthOfObject, literal(type(_,S1x))),atom_to_term(S1x,S1X,_),
   rdf(Hand, knowrob:widthOfObject, literal(type(_,S1y))),atom_to_term(S1y,S1Y,_),
   rdf(Hand, knowrob:heightOfObject, literal(type(_,S1z))),atom_to_term(S1z,S1Z,_),
   Size = [S1X, S1Y, S1Z].


%핸드 사이즈
handSize(Hand, HandSize):-
	rdfs_individual_of(Hand,  knowrob:'LeftHand'),
	rdf_has(Hand, knowrob:depthOfObject, Depth),
	rdf_has(Hand, knowrob:widthOfObject, Width),
	rdf_has(Hand, knowrob:heightOfObject, Height),
	set([Depth,Width,Height],HandSize).

currentObjectPerception(Object, CurrentPerception) :-
   rdfs_individual_of(Object,  knowrob:'Artifact'),
   not(rdf(Object, rdf:type,  knowrob:'Hand')),
      latest_detection_of_instance(Object, CurrentPerception).

currentFingerTipTouchPerception(Finger, TouchPerception) :-
      (rdfs_individual_of(Finger,knowrob:'UrdfLink'),
      rdfs_individual_of(Finger,  knowrob:'FingerTip')),
      latest_detection_of_touchPerception(Finger, TouchPerception).
      
currentFingerTipTorque(Finger, Torque) :-
      currentFingerTipTouchPerception(Finger, TouchPerception),
      rdf_has(TouchPerception, knowrob:torque, Torque).

currentFingerTipForce(Finger, Force) :-
      currentFingerTipTouchPerception(Finger, TouchPerception),
      rdf_has(TouchPerception, knowrob:force, Force).

currentTipContactPerception(TipList) :-
      (rdfs_individual_of(Subject,  knowrob:'UrdfLink'),
      rdfs_individual_of(Subject,  knowrob:'Finger')),
      append([], Subject, TipList).

currentContactPerception(PerceptionList) :-
      (rdfs_individual_of(Subject,  knowrob:'UrdfLink'),
      rdfs_individual_of(Subject,  knowrob:'Finger')),
      latest_detection_of_touchPerception(Subject, TouchPerception),
      append([], TouchPerception, PerceptionList).

/*

currentFingerContactPerception(Subject, TouchPerception) :-
      (rdfs_individual_of(Subject,  knowrob:'UrdfLink'),
      rdfs_individual_of(Subject,  knowrob:'Finger')),
      latest_detection_of_touchPerception(Subject, TouchPerception).
*/

set([], []).
set([H|T], [H|T1]) :- subtract(T, [H], T2), set(T2, T1).

currentHandTouchPerception(Hand) :-
      rdfs_individual_of(Hand,  knowrob:'Hand'),
      findall(TouchPerception, (rdf_has(Hand,'http://knowrob.org/kb/srdl2-comp.owl#subComponent',Fingertip),currentFingerTipTouchPerception(FingerTip, TouchPerception)), TouchPerceptions),
      set(TouchPerceptions,X),
      counter(X, Y),
      Y is 3 ,
      write(X),
      write(Y).

findComponent(Hand) :-
   rdfs_individual_of(Hand,  knowrob:'Hand'),
   findall(TouchPerception, (rdf_has(Hand,'http://knowrob.org/kb/srdl2-comp.owl#subComponent',Fingertip),currentFingerTipTouchPerception(FingerTip, TouchPerception)), TouchPerceptions),
   set(TouchPerceptions,X),
   write(X).

counter([],0).
counter([_|T],N) :-
      counter(T,M),
      N is M+1 .
       
   


currentHandTorque(Hand, Torques) :-
      rdfs_individual_of(Hand,  knowrob:'Hand'),
      findall(Torque, (rdf_has(Object, knowrob:objectActedOn, Fingertip), rdfs_individual_of(Fingertip, knowrob:'FingerTip'),currentFingerTipTorque(Fingertip,Torque)),Torques).
      
currentHandForce(Hand, Forces) :-
      rdfs_individual_of(Hand,  knowrob:'Hand'),
      findall(Force, (rdf_has(Object, knowrob:objectActedOn, Fingertip), rdfs_individual_of(Fingertip, knowrob:'FingerTip'),currentFingerTipForce(Fingertip,Force)),Forces).

	  
/*
      (rdfs_individual_of(Finger, knowrob:'UrdfLink'),
      rdfs_individual_of(Finger, knowrob:'Finger')),
      currentFingerContactPerception(Finger, TouchPerception),
      findall([StartTime, Object, EventOccurAt, Torque], (rdf_has(TouchPerception, knowrob:startTime, StartTime), rdf_has(TouchPerception, knowrob:objectActedOn, Object), 
      rdf_has(TouchPerception, knowrob:eventOccursAt, EventOccurAt),rdf_has(TouchPerception, knowrob:torque, Torque)), PerceptionList).
*/



currentHandPerception(Hand, CurrentPerception):-
   rdf(Hand, rdf:type,  knowrob:'LeftHand'),
   %rdf(FingerTip, rdf:type, knowrob:'FingerTip'),
   rdf(CurrentPerception,knowrob:objectActedOn, Hand).
   %latest_detection_of_instance(Object, CurrentPerception).

currentJointPerception(Joint, CurrentPerception) :-
   rdfs_individual_of(Joint, 'http://knowrob.org/kb/srdl2-comp.owl#FixedUrdfJoint'),
   latest_detection_of_joint(Joint, CurrentPerception).
   
currentTouchPerception(Hand, CurrentPerception) :-
	rdf(Hand, rdf:type,  knowrob:'LeftHand'),
	currentFingerTipTouchPerception(FingerTip, CurrentPerception).

currentObjectPose(Object, Pose) :-
   currentObjectPerception(Object, CurrentPerception),
   rdf(CurrentPerception, knowrob:eventOccursAt, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m02',literal(type(_,O1a))),atom_to_term(O1a,O1A,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m12',literal(type(_,O1b))),atom_to_term(O1b,O1B,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m22',literal(type(_,O1c))),atom_to_term(O1c,O1C,_),
   Pose = [P1X, P1Y, P1Z, O1A, O1B, O1C].

 %이거 하는중
currentHandPose(Hand, Pose) :-
   currentHandPerception(Hand, CurrentPerception),
   rdf(CurrentPerception, knowrob:eventOccursAt, Object_Matrix),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,P1x))),atom_to_term(P1x,P1X,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,P1y))),atom_to_term(P1y,P1Y,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,P1z))),atom_to_term(P1z,P1Z,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m02',literal(type(_,O1a))),atom_to_term(O1a,O1A,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m12',literal(type(_,O1b))),atom_to_term(O1b,O1B,_),
   rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m22',literal(type(_,O1c))),atom_to_term(O1c,O1C,_),
   Pose = [P1X, P1Y, P1Z, O1A, O1B, O1C].


currentJointAngle(Joint, Angle):-
   currentJointPerception(Joint, CurrentPerception),
   rdf(CurrentPerception,knowrob:radius,literal(type(_,J1r))),atom_to_term(J1r,J1R,_),
   Angle = J1R.
   
currentJointVelocity(Joint, Velocity):-
   currentJointPerception(Joint, CurrentPerception),
   rdf(CurrentPerception,knowrob:velocity,literal(type(_,J1v))),atom_to_term(J1v,J1V,_),
   Velocity = J1V.

currentJointEffort(Joint, Effort):-
   currentJointPerception(Joint, CurrentPerception),
   rdf(CurrentPerception,knowrob:effort,literal(type(_,J1e))),atom_to_term(J1e,J1E,_),
   Effort = J1E.
   
currentJointTorque(Hand, Torque):-
	currentTouchPerception(Hand, CurrentPerception),
   rdf(CurrentPerception,knowrob:torque,literal(type(_,J1t))),atom_to_term(J1t,J1T,_),
   Torque = J1T.


tilted_Object(Object):-
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
   (RadiusX>10,RadiusX<350),
   (RadiusY>10,RadiusY<350).




standing_Object(Object):-
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
   (RadiusX=<10;RadiusX>=350),
   (RadiusY=<10;RadiusY>=350).




currentObjectInfo(ObjectPose, ObjectSize, ObjectPoint):-
   nth0(0, ObjectPose, ObjectPoseX),
   nth0(1, ObjectPose, ObjectPoseY),
   nth0(2, ObjectPose, ObjectPoseZ),
   nth0(0, ObjectSize, ObjectSizeX),
   nth0(1, ObjectSize, ObjectSizeY),
   nth0(2, ObjectSize, ObjectSizeZ),
   P1X is ObjectPoseX + (ObjectSizeX/2),
   P1Y is ObjectPoseY - (ObjectSizeY/2),
   P1Z is ObjectPoseZ + (ObjectSizeZ/2),
   P1 = [P1X, P1Y, P1Z],
   P2X is ObjectPoseX + (ObjectSizeX/2),
   P2Y is ObjectPoseY + (ObjectSizeY/2),
   P2Z is ObjectPoseZ + (ObjectSizeZ/2),
   P2 = [P2X, P2Y, P2Z],
   P3X is ObjectPoseX - (ObjectSizeX/2),
   P3Y is ObjectPoseY - (ObjectSizeY/2),
   P3Z is ObjectPoseZ + (ObjectSizeZ/2),
   P3 = [P3X, P3Y, P3Z],
   P4X is ObjectPoseX - (ObjectSizeX/2),
   P4Y is ObjectPoseY + (ObjectSizeY/2),
   P4Z is ObjectPoseZ + (ObjectSizeZ/2),
   P4 = [P4X, P4Y, P4Z],
   P5X is ObjectPoseX + (ObjectSizeX/2),
   P5Y is ObjectPoseY - (ObjectSizeY/2),
   P5Z is ObjectPoseZ - (ObjectSizeZ/2),
   P5 = [P5X, P5Y, P5Z],
   P6X is ObjectPoseX + (ObjectSizeX/2),
   P6Y is ObjectPoseY + (ObjectSizeY/2),
   P6Z is ObjectPoseZ - (ObjectSizeZ/2),
   P6 = [P6X, P6Y, P6Z],
   P7X is ObjectPoseX - (ObjectSizeX/2),
   P7Y is ObjectPoseY - (ObjectSizeY/2),
   P7Z is ObjectPoseZ - (ObjectSizeZ/2),
   P7 = [P7X, P7Y, P7Z],
   P8X is ObjectPoseX - (ObjectSizeX/2),
   P8Y is ObjectPoseY + (ObjectSizeY/2),
   P8Z is ObjectPoseZ - (ObjectSizeZ/2),
   P8 = [P8X, P8Y, P8Z],
   ObjectPoint = [P1, P2, P3, P4, P5, P6, P7, P8].

intersects(Hand, Object, IntersectPer):-
   rdfs_individual_of(Hand,  knowrob:'LeftHand'),
   rdfs_individual_of(Object,  knowrob:'Artifact'),
   not(rdf(Object, rdf:type,  knowrob:'LeftHand')),
   currentObjectPose(Object, ObjectPose),
   objectSize(Object, ObjectSize),
   currentObjectInfo(ObjectPose, ObjectSize, ObjectPoint),
   currentHandPose(Hand, HandPose),
   %handSize(Hand, HandSize),
   leftHandSize(Hand, HandSize),
   %handPoint구하기 
   currentObjectInfo(HandPose, HandSize, HandPoint),
   
   
   nth0(1, HandPoint, Hand1), nth0(0, Hand1, X1), nth0(1, Hand1, Y1), nth0(2, Hand1, Z1),
   nth0(6, HandPoint, Hand2), nth0(0, Hand2, X2), nth0(1, Hand2, Y2), nth0(2, Hand2, Z2),
   nth0(1, ObjectPoint, Object3), nth0(0, Object3, X3), nth0(1, Object3, Y3), nth0(2, Object3, Z3),
   nth0(6, ObjectPoint, Object4), nth0(0, Object4, X4), nth0(1, Object4, Y4), nth0(2, Object4, Z4),
   once((X2>X3, X2>X4, LengthX is 0);
   (X2<X3, X1>X3, X2>X4, LengthX is X3-X2);
   (X2<X3, X1>X3, X2<X4, X1>X4, LengthX is X3-X4);
   (X1<X3, X2>X4, LengthX is X1-X2);
   (X1<X3, X1>X4, X2<X4, LengthX is X1-X4);
   (X1<X3, X1<X4, LengthX is 0)),
   once((Y2>Y3, Y2>Y4, LengthY is 0);
   (Y2<Y3, Y1>Y3, Y2>Y4, LengthY is Y3-Y2);
   (Y2<Y3, Y1>Y3, Y2<Y4, Y1>Y4, LengthY is Y3-Y4);
   (Y1<Y3, Y2>Y4, LengthY is Y1-Y2);
   (Y1<Y3, Y1>Y4, Y2<Y4, LengthY is Y1-Y4);
   (Y1<Y3, Y1<Y4, LengthY is 0)),
   once((Z2>Z3, Z2>Z4, LengthZ is 0);
   (Z2<Z3, Z1>Z3, Z2>Z4, LengthZ is Z3-Z2);
   (Z2<Z3, Z1>Z3, Z2<Z4, Z1>Z4, LengthZ is Z3-Z4);
   (Z1<Z3, Z2>Z4, LengthZ is Z1-Z2);
   (Z1<Z3, Z1>Z4, Z2<Z4, LengthZ is Z1-Z4);
   (Z1<Z3, Z1<Z4, LengthZ is 0)),
   HandLengthX is X1-X2, HandLengthY is Y1-Y2, HandLengthZ is Z1-Z2,
   HandArea is HandLengthX*HandLengthY*HandLengthZ,
   IntersectArea is LengthX*LengthY*LengthZ,
   IntersectPer is IntersectArea/HandArea,
   IntersectPer > 0.01.
   
not_intersects(Hand, Object, IntersectPer):-
   rdfs_individual_of(Hand,  knowrob:'Hand'),
   rdfs_individual_of(Object,  knowrob:'Artifact'),
   not(rdf(Object, rdf:type,  knowrob:'Hand')),
   currentObjectPose(Object, ObjectPose),
   objectSize(Object, ObjectSize),
   currentObjectInfo(ObjectPose, ObjectSize, ObjectPoint),
   currentHandPose(Hand, HandPose),
   handSize(Hand, HandSize),
   currentObjectInfo(HandPose, HandSize, HandPoint),
   nth0(1, HandPoint, Hand1), nth0(0, Hand1, X1), nth0(1, Hand1, Y1), nth0(2, Hand1, Z1),
   nth0(6, HandPoint, Hand2), nth0(0, Hand2, X2), nth0(1, Hand2, Y2), nth0(2, Hand2, Z2),
   nth0(1, ObjectPoint, Object3), nth0(0, Object3, X3), nth0(1, Object3, Y3), nth0(2, Object3, Z3),
   nth0(6, ObjectPoint, Object4), nth0(0, Object4, X4), nth0(1, Object4, Y4), nth0(2, Object4, Z4),
   once((X2>X3, X2>X4, LengthX is 0);
   (X2<X3, X1>X3, X2>X4, LengthX is X3-X2);
   (X2<X3, X1>X3, X2<X4, X1>X4, LengthX is X3-X4);
   (X1<X3, X2>X4, LengthX is X1-X2);
   (X1<X3, X1>X4, X2<X4, LengthX is X1-X4);
   (X1<X3, X1<X4, LengthX is 0)),
   once((Y2>Y3, Y2>Y4, LengthY is 0);
   (Y2<Y3, Y1>Y3, Y2>Y4, LengthY is Y3-Y2);
   (Y2<Y3, Y1>Y3, Y2<Y4, Y1>Y4, LengthY is Y3-Y4);
   (Y1<Y3, Y2>Y4, LengthY is Y1-Y2);
   (Y1<Y3, Y1>Y4, Y2<Y4, LengthY is Y1-Y4);
   (Y1<Y3, Y1<Y4, LengthY is 0)),
   once((Z2>Z3, Z2>Z4, LengthZ is 0);
   (Z2<Z3, Z1>Z3, Z2>Z4, LengthZ is Z3-Z2);
   (Z2<Z3, Z1>Z3, Z2<Z4, Z1>Z4, LengthZ is Z3-Z4);
   (Z1<Z3, Z2>Z4, LengthZ is Z1-Z2);
   (Z1<Z3, Z1>Z4, Z2<Z4, LengthZ is Z1-Z4);
   (Z1<Z3, Z1<Z4, LengthZ is 0)),
   HandLengthX is X1-X2, HandLengthY is Y1-Y2, HandLengthZ is Z1-Z2,
   HandArea is HandLengthX*HandLengthY*HandLengthZ,
   IntersectArea is LengthX*LengthY*LengthZ,
   IntersectPer is IntersectArea/HandArea,
   IntersectPer =< 0.01.
   
empty_Hand(Hand):-
   rdfs_individual_of(Hand,  knowrob:'Hand'),
   findall(Object,  (rdfs_individual_of(Object,  knowrob:'Artifact'), not(rdf(Object, rdf:type,  knowrob:'Hand'))) ,    Objects),
   foreach(member(O,Objects),
   not_intersects(Hand, O, IntersectPer)).


full_Hand(Hand, Object):-
   intersects(Hand, Object, IntersectPer),
   IntersectPer > 0.03.

check_Effort(Hand):-
	rdf(Hand, rdf:type,  knowrob:'LeftHand'),
	findall(Joint,  rdf(Hand, 'http://knowrob.org/kb/srdl2-comp.owl#subComponent',  Joint),    Joints),
	foreach(member(J,Joints), (currentJointEffort(Joint, Effort), Effort > 0.0)).

check_Velocity(Hand):-
	rdf(Hand, rdf:type,  knowrob:'LeftHand'),
	findall(Joint,  rdf(Hand, 'http://knowrob.org/kb/srdl2-comp.owl#subComponent',  Joint),    Joints),
	foreach(member(J,Joints), (currentJointVelocity(Joint, Velocity), Velocity =< 0.0)).
	
opened_Hand(Hand):-
   rdf(Hand, rdf:type,  knowrob:'LeftHand'),
   findall(Joint,  rdf(Hand, 'http://knowrob.org/kb/srdl2-comp.owl#subComponent',  Joint),    Joints),
   (member(J,Joints), (currentJointAngle(J, Angle), Angle < 0.2)),
   forall(member(J,Joints), (currentJointTorque(Hand, Torque), Torque < 0.1)).

closed_Hand(Hand):-
   rdf(Hand, rdf:type,  knowrob:'LeftHand'),
   findall(Joint,  rdf(Hand, 'http://knowrob.org/kb/srdl2-comp.owl#subComponent',  Joint),    Joints),
   not(forall(member(J,Joints), (currentJointAngle(J, Angle), Angle < 0.2))),
   not(forall(member(J,Joints), (currentJointTorque(Hand, Torque), Torque < 0.1))).


graspedBy(Hand, Object):-
   closed_Hand(Hand),
   full_Hand(Hand, Object).

graspedBy(Hand):-
   closed_Hand(Hand),
   check_Velocity(Hand),
   check_Effort(Hand),
   currentHandTouchPerception(Hand).

intersects2(Hand, Object, LengthX, LengthY, LengthZ, HandArea, IntersectArea, IntersectPer):-
   rdfs_individual_of(Hand,  knowrob:'Hand'),
   rdfs_individual_of(Object,  knowrob:'Artifact'),
   not(rdf(Object, rdf:type,  knowrob:'Hand')),
   currentObjectPose(Object, ObjectPose),
   objectSize(Object, ObjectSize),
   currentObjectInfo(ObjectPose, ObjectSize, ObjectPoint),
   currentHandPose(Hand, HandPose),
   handSize(Hand, HandSize),
   currentObjectInfo(HandPose, HandSize, HandPoint),
   nth0(1, HandPoint, Hand1), nth0(0, Hand1, X1), nth0(1, Hand1, Y1), nth0(2, Hand1, Z1),
   nth0(6, HandPoint, Hand2), nth0(0, Hand2, X2), nth0(1, Hand2, Y2), nth0(2, Hand2, Z2),
   nth0(1, ObjectPoint, Object3), nth0(0, Object3, X3), nth0(1, Object3, Y3), nth0(2, Object3, Z3),
   nth0(6, ObjectPoint, Object4), nth0(0, Object4, X4), nth0(1, Object4, Y4), nth0(2, Object4, Z4),
   once((X2>X3, X2>X4, LengthX is 0);
   (X2<X3, X1>X3, X2>X4, LengthX is X3-X2);
   (X2<X3, X1>X3, X2<X4, X1>X4, LengthX is X3-X4);
   (X1<X3, X2>X4, LengthX is X1-X2);
   (X1<X3, X1>X4, X2<X4, LengthX is X1-X4);
   (X1<X3, X1<X4, LengthX is 0)),
   once((Y2>Y3, Y2>Y4, LengthY is 0);
   (Y2<Y3, Y1>Y3, Y2>Y4, LengthY is Y3-Y2);
   (Y2<Y3, Y1>Y3, Y2<Y4, Y1>Y4, LengthY is Y3-Y4);
   (Y1<Y3, Y2>Y4, LengthY is Y1-Y2);
   (Y1<Y3, Y1>Y4, Y2<Y4, LengthY is Y1-Y4);
   (Y1<Y3, Y1<Y4, LengthY is 0)),
   once((Z2>Z3, Z2>Z4, LengthZ is 0);
   (Z2<Z3, Z1>Z3, Z2>Z4, LengthZ is Z3-Z2);
   (Z2<Z3, Z1>Z3, Z2<Z4, Z1>Z4, LengthZ is Z3-Z4);
   (Z1<Z3, Z2>Z4, LengthZ is Z1-Z2);
   (Z1<Z3, Z1>Z4, Z2<Z4, LengthZ is Z1-Z4);
   (Z1<Z3, Z1<Z4, LengthZ is 0)),
   HandLengthX is X1-X2, HandLengthY is Y1-Y2, HandLengthZ is Z1-Z2,
   HandArea is HandLengthX*HandLengthY*HandLengthZ,
   IntersectArea is LengthX*LengthY*LengthZ,
   IntersectPer is IntersectArea/HandArea.






latest_detection_of_instance(Object, LatestDetection) :-

  ((rdf_has(Object, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Object,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              (rdfs_individual_of(D_i,  knowrob:'SemanticMapPerception'); rdfs_individual_of(D_i,  knowrob:'VisualPerception')),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),
	
    % compute the homography for the newest perception
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).














  latest_detection_of_touchPerception(Subject, LatestDetection) :-

  ((rdf_has(Subject, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Subject,St], (rdf_has(D_i, knowrob:objectActedOn, Subject),
                              ( rdfs_individual_of(D_i,  knowrob:'TouchPerception')),
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


%% detection_endtime(+Detection, -EndTime) is nondet.
%
% Determine the end time of an object detection as numerical value.
% If the knowrob:endTime is asserted, it is read and and transformed
% into a numeric value. Otherwise, the predicate searches for later
% perceptions of the same object and takes the startTime of the first
% subsequent detection as the endTime of the current detection. If
% there is neither an asserted endTime nor any later detection of the
% object, it is assumed that the observation is still valid and the
% current time + 1s is returned (to avoid problems with time glitches).
%
% @param Detection  Instance of an event
% @param EndTime    Numeric value describing the ent time
%
detection_endtime(Detection, EndTime) :-

  % end time is asserted
  rdf_triple(knowrob:endTime, Detection, EndTtG),
  rdf_split_url(_, EndTt, EndTtG),
  atom_concat('timepoint_', EndTAtom, EndTt),
  term_to_atom(EndTime, EndTAtom),!;

  % search for later detections of the object
  ( rdf_has(LaterDetection, knowrob:previousDetectionOfObject, Detection),
    rdf_triple(knowrob:startTime, LaterDetection, EndTtG),
    rdf_split_url(_, EndTt, EndTtG),
    atom_concat('timepoint_', EndTAtom, EndTt),
    term_to_atom(EndTime, EndTAtom),! );

  % check if the object has been destroyed in the meantime
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
% @param P1     List [_, _, Time] as used in latest_detection_of_instance, latest_detection_of_type, latest_inferred_object_set
% @param P2     List [_, _, Time] as used in latest_detection_of_instance, latest_detection_of_type, latest_inferred_object_set
%
compare_object_detections(Delta, P1, P2) :-

    nth0(2, P1, St1),
    nth0(2, P2, St2),
    compare(Delta, St2, St1).
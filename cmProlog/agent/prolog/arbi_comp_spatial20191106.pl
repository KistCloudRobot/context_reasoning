/** <module> Predicates for spatial reasoning

  This module contains all computables that calculate qualitative spatial relations
  between objects to allow for spatial reasoning. In addition, there are computables
  to extract components of a matrix or position vector.

  Copyright (C) 2009-13 Moritz Tenorth, Lars Kunze
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@author Moritz Tenorth, Lars Kunze
@license BSD

*/
:- module(arbi_comp_spatial,
    [
   % holds/2,
    comp_graspedBy/2,
    inFrontAreaOfRoom/2,
    insideAreaOfRoom/2,
    insideOfRoom/2,
    near/2,
    %on_Physical/2,
    %on_Physical_at_time/3,
    %test_physical/3,
    angleForLookAt/4,
    comp_locatedInArtifactContainer/2,
    latestObjectXpoint/2,
    latestObjectYpoint/2,
    latestObjectZpoint/2,
    comp_inFrontAreaOfRoom/2,
    comp_insideAreaOfRoom/2,
    locatedInRoom/2,
    personLocatedInRoom/2,
    lookAt/2,
    notFollowRobot/2,
    comp_csd/3,
    comp_RCCD_C/4,
    comp_RCCD_C2/3,
    comp_RCCD_C3/4,
    comp_RCCD_P/4,
    comp_RCCD_P2/10,
    comp_RCCD_Hybrid/3,
    create_objs/1,
    comp_identical/3,
    rotation/7

    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_objects')).
:- use_module(library('knowrob_owl')).




:- rdf_db:rdf_register_ns(knowrob,      'http://knowrob.org/kb/knowrob.owl#',      [keep(true)]).
:- rdf_db:rdf_register_ns(arbi, 'http://www.arbi.com/ontologies/arbi.owl#',     [keep(true)]).


:- meta_predicate holds(0, ?, ?).
:- discontiguous holds/2.
:-  rdf_meta
    holds(:, r),
    comp_equal(r,r),
    latest_detection_of_instance(r,r).

on_Physical(Top, Bottom) :-
    get_timepoint(Instant),
    on_Physical_at_time(Top, Bottom, Instant).


on_Physical_at_time(Top, Bottom, Instant) :-

       rdfs_individual_of(Top,  knowrob:'SpatialThing-Localized'),
       rdfs_individual_of(Bottom,  knowrob:'SpatialThing-Localized'),
       Top \= Bottom,

       %rdf_assert(StartTime, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#TimePoint', query),
       %rdf_assert(EndTime, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#TimePoint', query),
       rdf_triple('http://knowrob.org/kb/knowrob.owl#objectActedOn' ,TVP, Top),
       rdf_triple('http://knowrob.org/kb/knowrob.owl#objectActedOn', BVP, Bottom),

       rdf(TVP, 'http://knowrob.org/kb/knowrob.owl#startTime', TstartTime),
       rdf(TVP, 'http://knowrob.org/kb/knowrob.owl#endTime', TendTime),
       
       %rdf_assert(TstartTime, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#TimePoint', query),
       %rdf_assert(TendTime, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#TimePoint', query),

       rdf(BVP, 'http://knowrob.org/kb/knowrob.owl#startTime', BstartTime),
       rdf(BVP, 'http://knowrob.org/kb/knowrob.owl#endTime', BendTime),

       rdf(TVP,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', TMatrix),
	rdf(BVP,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', BMatrix),
	rdf(TMatrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,Tx))),atom_to_term(Tx,TX,_),
	rdf(BMatrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,Bx))),atom_to_term(Bx,BX,_),
	rdf(TMatrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,Ty))),atom_to_term(Ty,TY,_),
	rdf(BMatrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,By))),atom_to_term(By,BY,_),
	rdf(TMatrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,Tz))),atom_to_term(Tz,TZ,_),
	rdf(BMatrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,Bz))),atom_to_term(Bz,BZ,_),
	rdf(Bottom,'http://knowrob.org/kb/knowrob.owl#widthOfObject',literal(type(_,Bw))),atom_to_term(Bw,BW,_),
	rdf(Bottom,'http://knowrob.org/kb/knowrob.owl#depthOfObject',literal(type(_,Bd))),atom_to_term(Bd,BD,_),
	rdf(Bottom,'http://knowrob.org/kb/knowrob.owl#heightOfObject',literal(type(_,Bh))),atom_to_term(Bh,BH,_),
	rdf(Top,'http://knowrob.org/kb/knowrob.owl#heightOfObject',literal(type(_,Th))),atom_to_term(Th,TH,_),
      <( BZ, TZ ),
      abs((TZ-(TH*0.5)) - (BZ+(BH*0.5)))<0.05,
      
      print(BX+" "+BW+" "+TX),
      =<( (BX - (BW*0.5)), TX ), >=( (BX + (BW*0.5)), TX ),
      =<( (BY - 0.5*BW), TY ), >=( (BY + 0.5*BW), TY ),
 

       %rdf_split_url(_, InstantLocal, Instant),
       %atom_concat('timepoint_', InstantTimeAtom, InstantLocal),
       %term_to_atom(ITime, InstantTimeAtom),
       
       rdf_split_url(_, TstartTimeLocal, TstartTime),
       atom_concat('timepoint_', TstartTimeAtom, TstartTimeLocal),
       term_to_atom(TsTime, TstartTimeAtom),
       
       rdf_split_url(_, TendTimeLocal, TendTime),
       atom_concat('timepoint_', TendTimeAtom, TendTimeLocal),
       term_to_atom(TeTime, TendTimeAtom),
       
       rdf_split_url(_, BstartTimeLocal, BstartTime),
       atom_concat('timepoint_', BstartTimeAtom, BstartTimeLocal),
       term_to_atom(BsTime, BstartTimeAtom),
       
       rdf_split_url(_, BendTimeLocal, BendTime),
       atom_concat('timepoint_', BendTimeAtom, BendTimeLocal),
       term_to_atom(BeTime, BendTimeAtom),
       
       Instant>=TsTime,
       Instant=<TeTime,
       Instant>=BsTime,
       Instant=<BeTime,
       
       print(TsTime+" "),
       print(TeTime+" "),
       print(BsTime+" "),
       print(BeTime+" "),
       print(Instant).
 


test_physical(Top, Bottom, Instant) :-
	%rdf(Instant,   rdf:type, knowrob:'TimePoint'),
	rdfs_individual_of(Top,  knowrob:'SpatialThing-Localized'),
       rdfs_individual_of(Bottom,  knowrob:'SpatialThing-Localized'),
       Top \= Bottom,

       %rdf_assert(StartTime, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#TimePoint', query),
       %rdf_assert(EndTime, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#TimePoint', query),
       rdf_triple('http://knowrob.org/kb/knowrob.owl#objectActedOn' ,TVP, Top),
       rdf_triple('http://knowrob.org/kb/knowrob.owl#objectActedOn', BVP, Bottom),

       rdf(TVP, 'http://knowrob.org/kb/knowrob.owl#startTime', TstartTime),
       rdf(TVP, 'http://knowrob.org/kb/knowrob.owl#endTime', TendTime),
       
       %rdf_assert(TstartTime, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#TimePoint', query),
       %rdf_assert(TendTime, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#TimePoint', query),

       rdf(BVP, 'http://knowrob.org/kb/knowrob.owl#startTime', BstartTime),
       rdf(BVP, 'http://knowrob.org/kb/knowrob.owl#endTime', BendTime),

       rdf(TVP,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', TMatrix),
	rdf(BVP,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', BMatrix),
	rdf(TMatrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,Tx))),atom_to_term(Tx,TX,_),
	rdf(BMatrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,Bx))),atom_to_term(Bx,BX,_),
	rdf(TMatrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,Ty))),atom_to_term(Ty,TY,_),
	rdf(BMatrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,By))),atom_to_term(By,BY,_),
	rdf(TMatrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,Tz))),atom_to_term(Tz,TZ,_),
	rdf(BMatrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,Bz))),atom_to_term(Bz,BZ,_),
	rdf(Bottom,'http://knowrob.org/kb/knowrob.owl#widthOfObject',literal(type(_,Bw))),atom_to_term(Bw,BW,_),
	rdf(Bottom,'http://knowrob.org/kb/knowrob.owl#depthOfObject',literal(type(_,Bd))),atom_to_term(Bd,BD,_),
	rdf(Bottom,'http://knowrob.org/kb/knowrob.owl#heightOfObject',literal(type(_,Bh))),atom_to_term(Bh,BH,_),
	rdf(Top,'http://knowrob.org/kb/knowrob.owl#heightOfObject',literal(type(_,Th))),atom_to_term(Th,TH,_),
      <( BZ, TZ ),
      abs((TZ-(TH*0.5)) - (BZ+(BH*0.5)))<0.05,
      
      print(BX+" "+BW+" "+TX),
      =<( (BX - (BW*0.5)), TX ), >=( (BX + (BW*0.5)), TX ),
      =<( (BY - 0.5*BW), TY ), >=( (BY + 0.5*BW), TY ).

     

       





%% holds(+OnPhysicalPred:compound, +T) is nondet.
%
% Usage: holds(on_Physical(?Top, ?Bottom), +T)
%
% Check if Top has been in the area of and above Bottom at time point T.
%
% Currently does not take the orientation into account, only the position and dimension.
%
% @param Top    Identifier of the upper Object
% @param Bottom Identifier of the lower Object
% @param T      TimePoint or Event for which the relations is supposed to hold
%
holds(on_Physical(Top, Bottom),T) :-
      rdfs_individual_of(Top,  knowrob:'SpatialThing-Localized'),
      rdfs_individual_of(Bottom,  knowrob:'SpatialThing-Localized'),
      not(rdfs_individual_of(Top,  knowrob:'Water')),
      not(rdfs_individual_of(Bottom,  knowrob:'Water')),
      latest_detection_of_instance(Top, TP),
      latest_detection_of_instance(Bottom, BP),
      rdf(TP,'http://knowrob.org/kb/knowrob.owl#eventOccursAt',TMatrix),
	rdf(BP,'http://knowrob.org/kb/knowrob.owl#eventOccursAt',BMatrix),
	rdf(TMatrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,Tx))),atom_to_term(Tx,TX,_),
	rdf(BMatrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,Bx))),atom_to_term(Bx,BX,_),
	rdf(TMatrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,Ty))),atom_to_term(Ty,TY,_),
	rdf(BMatrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,By))),atom_to_term(By,BY,_),
	rdf(TMatrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,Tz))),atom_to_term(Tz,TZ,_),
	rdf(BMatrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,Bz))),atom_to_term(Bz,BZ,_),
	rdf(Bottom,'http://knowrob.org/kb/knowrob.owl#depthOfObject',literal(type(_,Bw))),atom_to_term(Bw,BW,_),
	rdf(Bottom,'http://knowrob.org/kb/knowrob.owl#widthOfObject',literal(type(_,Bd))),atom_to_term(Bd,BD,_),
	rdf(Bottom,'http://knowrob.org/kb/knowrob.owl#heightOfObject',literal(type(_,Bh))),atom_to_term(Bh,BH,_),
	rdf(Top,'http://knowrob.org/kb/knowrob.owl#heightOfObject',literal(type(_,Th))),atom_to_term(Th,TH,_),
      <( BZ, TZ ),
      abs((TZ-0.5*TH) - (BZ+0.5*BH))<0.05,
   %   print('check1'),

     % additional criterion: center of the top entity has to be inside the
     % area of the bottom entity
     =<( (BX - 0.5*BD), TX ), >=( (BX + 0.5*BD), TX ),
     =<( (BY - 0.5*BW), TY ), >=( (BY + 0.5*BW), TY ),
     Top \= Bottom.




comp_graspedBy(Object, Hand) :-
	print('ok').

comp_locatedInArtifactContainer(Object, Container) :-
	get_timepoint(T),
	rdfs_individual_of(Container,  knowrob:'ContainerArtifact'),
    holds(in_ContGeneric(Object, Container), T).


create_objs(N) :-
   N>0,
   atom_concat('http://knowrob.org/kb/knowrob.owl#Test', N, ObIns),
   rdf_assert(ObIns, 'http://knowrob.org/kb/knowrob.owl#type', 'http://knowrob.org/kb/knowrob.owl#Desk-PieceOfFurniture'),
   atom_concat('http://knowrob.org/kb/knowrob.owl#SemanticMapPerception', N, PIns),
   rdf_assert(PIns, 'http://knowrob.org/kb/knowrob.owl#type', 'http://knowrob.org/kb/knowrob.owl#SemanticMapPerception'),
   rdf_assert(PIns, 'http://knowrob.org/kb/knowrob.owl#startTime', 'http://knowrob.org/kb/ias_semantic_map.owl#timepoint_0'),
   rdf_assert(PIns, 'http://knowrob.org/kb/knowrob.owl#objectActedOn', ObIns),
   atom_concat('http://knowrob.org/kb/knowrob.owl#rotationMatrix3D', N, RIns),
   rdf_assert(RIns, 'http://knowrob.org/kb/knowrob.owl#type', 'http://knowrob.org/kb/knowrob.owl#RotationMatrix3D'),
   random_between(-50,50,R),
   random(F),
   random_between(-50,50,R1),
   random(F1),
   M03 = R,
   M13 = R1,
   create_pose2([0,0,0,M03, 0,0,0,M13, 0,0,0,1, 0,0,0,0], RIns),
   N1 is N - 1,
   create_objs(N1).

create_pose2([M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33], PoseInst) :-

  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m00',literal(type(xsd:float, M00))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m01',literal(type(xsd:float, M01))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m02',literal(type(xsd:float, M02))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(xsd:float, M03))),

  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m10',literal(type(xsd:float, M10))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m11',literal(type(xsd:float, M11))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m12',literal(type(xsd:float, M12))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(xsd:float, M13))),

  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m20',literal(type(xsd:float, M20))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m21',literal(type(xsd:float, M21))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m22',literal(type(xsd:float, M22))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(xsd:float, M23))),

  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m30',literal(type(xsd:float, M30))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m31',literal(type(xsd:float, M31))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m32',literal(type(xsd:float, M32))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m33',literal(type(xsd:float, M33))).







 comp_RCCD_C(Object, Robot, D,RCCD) :-
    rdfs_individual_of(Object,  knowrob:'SpatialThing'),
    rdfs_individual_of(Robot,  knowrob:'SpatialThing'),
    rdf(Robot, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
    latest_detection_of_instance(Object, VPO),
    latest_location_detection_of_instance(Robot,VPR),
    rdf(VPO,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object_Matrix),
    rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Robot_Matrix),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O1x))),atom_to_term(O1x,OX,_),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O1y))),atom_to_term(O1y,OY,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O2x))),atom_to_term(O2x,RX,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O2y))),atom_to_term(O2y,RY,_),
    rotation(RX,RY,OX,OY,-D,X1,Y1),
    DX= RX - X1,
    DY= RY - Y1,
    Angle = (atan2(DX,DY) * (180/pi))+22.5,
    (Angle<0 -> Angle2 = Angle+360; Angle2=Angle),
    A = floor(((Angle2)/45.0)),
    once(
    ((DX=:=0,DY=:=0),RCCD='identical');
    (A=:=0 , RCCD='behind');
    (A=:=1 , RCCD='behindLeft');
    (A=:=2 , RCCD='left');
    (A=:=3 , RCCD='frontLeft');
    (A=:=4 , RCCD='front');
    (A=:=5 , RCCD='frontRight');
    (A=:=6 , RCCD='right');
    (A=:=7 , RCCD='behindRight')).


    notFollowRobot(Object, Robot) :-
    rdfs_individual_of(Object,  knowrob:'SpatialThing'),
    rdfs_individual_of(Robot,  knowrob:'SpatialThing'),
    rdf(Robot, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
    latest_detection_of_instance(Object, VPO),
    latest_location_detection_of_instance(Robot,VPR),


    rdf(VPO,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object_Matrix),
    rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Robot_Matrix),
    rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventHasValue',literal(type(_,A))),atom_to_term(A,D,_),

    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O1x))),atom_to_term(O1x,OX,_),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O1y))),atom_to_term(O1y,OY,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O2x))),atom_to_term(O2x,RX,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O2y))),atom_to_term(O2y,RY,_),

    Distance = sqrt((OX-RX)**2 + (OY -RY)**2),

    rotation(RX,RY,OX,OY,-D,X1,Y1),

    DX= X1 - RX,
    DY= Y1 - RY,
    atan2(DX,DY,A2),
    Angle = ( A2 * (180/pi)),
    (Angle<0 -> Angle2 = Angle+360; Angle2=Angle),
    once(
    (Angle2 < 202.5);
    (Angle2 >337.5);
    (Distance>2)
    ).

    angleForLookAt(Robot, Object , Direction, Angle3) :-
    rdfs_individual_of(Object,  knowrob:'SpatialThing'),
    rdfs_individual_of(Robot,  knowrob:'SpatialThing'),
    rdf(Robot, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
    latest_detection_of_instance(Object, VPO),
    latest_location_detection_of_instance(Robot,VPR),


    rdf(VPO,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object_Matrix),
    rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Robot_Matrix),
    rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventHasValue',literal(type(_,A))),atom_to_term(A,D,_),

    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O1x))),atom_to_term(O1x,OX,_),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O1y))),atom_to_term(O1y,OY,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O2x))),atom_to_term(O2x,RX,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O2y))),atom_to_term(O2y,RY,_),
   % print('sibal'),
    rotation(RX,RY,OX,OY,-D,X1,Y1),
    DX= X1 - RX,
    DY= Y1 - RY,
    atan2(DX,DY,A2),
    Angle = ( A2* (180/pi)),
    (Angle<0 -> Angle2 is Angle+360; Angle2 is Angle),



    once(
    ((DX=:=0,DY=:=0),RCCD='identical');
    (Angle2 >= 157.5, Angle2 <202.5 , RCCD='right',Angle3 = 90, Direction = 1);
    (Angle2 >= 202.5, Angle2 <247.5 , RCCD='behindRight',Angle3 = 135, Direction = 1);
    (Angle2 >= 247.5, Angle2 <292.5 , RCCD='behind',Angle3 = 180, Direction = 1);
    (Angle2 >= 292.5, Angle2 <337.5 , RCCD='behindLeft',Angle3 = 135, Direction = -1);
    (Angle2 >= 337.5; Angle2 <22.5 , RCCD='left',Angle3 = 90, Direction = -1);
    (Angle2 >= 22.5, Angle2 <67.5 , RCCD='frontLeft',Angle3 = 45, Direction = -1);
    (Angle2 >= 67.5, Angle2 <112.5 , RCCD='front',Angle3 = 0, Direction = 1);
    (Angle2 >= 112.5, Angle2 <157.5 , RCCD='frontRight',Angle3 = 45, Direction = 1)),
    print(Angle2).


    lookAt(Object, Robot) :-
    rdfs_individual_of(Object,  knowrob:'SpatialThing'),
    rdfs_individual_of(Robot,  knowrob:'SpatialThing'),
    rdf(Robot, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
    latest_detection_of_instance(Object, VPO),
    latest_location_detection_of_instance(Robot,VPR),


    rdf(VPO,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object_Matrix),
    rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Robot_Matrix),
    rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventHasValue',literal(type(_,A))),atom_to_term(A,D,_),

    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O1x))),atom_to_term(O1x,OX,_),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O1y))),atom_to_term(O1y,OY,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O2x))),atom_to_term(O2x,RX,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O2y))),atom_to_term(O2y,RY,_),

    rotation(RX,RY,OX,OY,-D,X1,Y1),

    DX= X1 - RX,
    DY= Y1 - RY,
    atan2(DX,DY,A2),

    Angle = ( A2 * (180/pi)),
    (Angle<0 -> Angle2 = Angle+360; Angle2=Angle),
    Angle2 >= 67.5, Angle2 <112.5.


near(Robot, Object) :-
    rdfs_individual_of(Object,  knowrob:'SpatialThing'),
    rdfs_individual_of(Robot,  knowrob:'SpatialThing'),
    rdf(Robot, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
    latest_detection_of_instance(Object, VPO),
    latest_location_detection_of_instance(Robot,VPR),


    rdf(VPO,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object_Matrix),
    rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Robot_Matrix),
    rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventHasValue',literal(type(_,A))),atom_to_term(A,D,_),

    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O1x))),atom_to_term(O1x,OX,_),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O1y))),atom_to_term(O1y,OY,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O2x))),atom_to_term(O2x,RX,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O2y))),atom_to_term(O2y,RY,_),

    Distance = sqrt((OX-RX)**2 + (OY -RY)**2),
    Distance<2.


 comp_csd(Object, Robot, B) :-
    rdfs_individual_of(Object,  knowrob:'SpatialThing'),
    rdfs_individual_of(Robot,  knowrob:'SpatialThing'),
    rdf(Robot, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
    latest_detection_of_instance(Object, VPO),
    latest_location_detection_of_instance(Robot,VPR),


    rdf(VPO,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object_Matrix),
    rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Robot_Matrix),


    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O1x))),atom_to_term(O1x,OX,_),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O1y))),atom_to_term(O1y,OY,_),

    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O2x))),atom_to_term(O2x,RX,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O2y))),atom_to_term(O2y,RY,_),


    DX= OX - RX,
    DY= OY - RY,



    Angle = (atan2(DX,DY) * (180/pi)),
    (Angle<0 -> Angle2 is Angle+360; Angle2 is Angle),
    print(Angle2),
    A = floor(((Angle2)/45.0)),
    once((A=:=0 , B='s');
    (A=:=1 , B='sw');
    (A=:=2 , B='w');
    (A=:=3 , B='nw');
    (A=:=4 , B='n');
    (A=:=5 , B='ne');
    (A=:=6 , B='e');
    (A=:=7 , B='se');
    B='eq'),
    Object1 \= Object2.



    comp_RCCD_C2(Object, Robot,RCCD) :-
    rdfs_individual_of(Object,  knowrob:'SpatialThing'),
    rdfs_individual_of(Robot,  knowrob:'SpatialThing'),
    rdf(Robot, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
    latest_detection_of_instance(Object, VPO),
    latest_location_detection_of_instance(Robot,VPR),


    rdf(VPO,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object_Matrix),
    rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Robot_Matrix),
    rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventHasValue',literal(type(_,A))),atom_to_term(A,D,_),

    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O1x))),atom_to_term(O1x,OX,_),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O1y))),atom_to_term(O1y,OY,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O2x))),atom_to_term(O2x,RX,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O2y))),atom_to_term(O2y,RY,_),
    print('sibal'),
    rotation(RX,RY,OX,OY,-D,X1,Y1),
    DX= X1 - RX,
    DY= Y1 - RY,
    atan2(DX,DY,A2),
    Angle = ( A2* (180/pi)),
    (Angle<0 -> Angle2 is Angle+360; Angle2 is Angle),



    once(
    ((DX=:=0,DY=:=0),RCCD='identical');
    (Angle2 >= 157.5, Angle2 <202.5 , RCCD='right');
    (Angle2 >= 202.5, Angle2 <247.5 , RCCD='behindRight');
    (Angle2 >= 247.5, Angle2 <292.5 , RCCD='behind');
    (Angle2 >= 292.5, Angle2 <337.5 , RCCD='behindLeft');
    (Angle2 >= 337.5; Angle2 <22.5 , RCCD='left');
    (Angle2 >= 22.5, Angle2 <67.5 , RCCD='frontLeft');
    (Angle2 >= 67.5, Angle2 <112.5 , RCCD='front');
    (Angle2 >= 112.5, Angle2 <157.5 , RCCD='frontRight')),
    print(Angle2).

check_FrontArea(CX, CY, TX, TY) :-
    getM(112.5,FM1),
    getM(67.5,FM2),
    getFragment(CX,CY,FM1,F1),
    getFragment(CX,CY,FM2,F2),
    TY-FM1*TX-F1>=0,TY-FM2*TX-F2>0.

check_LeftArea(CX, CY, TX, TY) :-
    getM(157.5,FM1),
    getM(202.5,FM2),
    getFragment(CX,CY,FM1,F1),
    getFragment(CX,CY,FM2,F2),
    TY-FM1*TX-F1<0,TY-FM2*TX-F2>=0.

check_RightArea(CX, CY, TX, TY) :-
    getM(22.5,FM1),
    getM(337.5,FM2),
    getFragment(CX,CY,FM1,F1),
    getFragment(CX,CY,FM2,F2),
    TY-FM1*TX-F1<0,TY-FM2*TX-F2>0.

check_BehindArea(CX, CY, TX, TY) :-
    getM(247.5,FM1),
    getM(292.5,FM2),
    getFragment(CX,CY,FM1,F1),
    getFragment(CX,CY,FM2,F2),
    TY-FM1*TX-F1<0,TY-FM2*TX-F2<0.

check_frontRightArea(CX, CY, TX, TY) :-
    getM(22.5,FM1),
    getM(67.5,FM2),
    getFragment(CX,CY,FM1,F1),
    getFragment(CX,CY,FM2,F2),
    TY-FM1*TX-F1>0,TY-FM2*TX-F2<0.

check_frontLeftArea(CX, CY, TX, TY) :-
    getM(112.5,FM1),
    getM(157.5,FM2),
    getFragment(CX,CY,FM1,F1),
    getFragment(CX,CY,FM2,F2),
    TY-FM1*TX-F1<0,TY-FM2*TX-F2>0.

check_behindLeftArea(CX, CY, TX, TY) :-
    getM(202.5,FM1),
    getM(247.5,FM2),
    getFragment(CX,CY,FM1,F1),
    getFragment(CX,CY,FM2,F2),
    TY-FM1*TX-F1<0,TY-FM2*TX-F2>0.

check_behindRightArea(CX, CY, TX, TY) :-
    getM(292.5,FM1),
    getM(337.5,FM2),
    getFragment(CX,CY,FM1,F1),
    getFragment(CX,CY,FM2,F2),
    TY-FM1*TX-F1>0,TY-FM2*TX-F2<0.


getRadian(D,R):-
	R=(pi*D)/180.
getM(D,M) :-
	R=pi*D/180,
	M=tan(R).
getFragment(XC,YC,M,F) :-
	F= YC-(M*XC).

rotation(XC,YC,X0,Y0,D,X1,Y1) :-
	getRadian(D,R),
	S=sin(R),
	C=cos(R),
	X1 is (X0-XC)*C-(Y0-YC)*S+XC,
	Y1 is (X0-XC)*S+(Y0-YC)*C+YC.


comp_RCCD_C3(Object, Robot, D,RCCD) :-
    rdfs_individual_of(Object,  knowrob:'Artifact'),
    rdfs_individual_of(Robot,  knowrob:'Artifact'),
    rdf(Robot, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
    latest_detection_of_instance(Object, VPO),
    latest_location_detection_of_instance(Robot,VPR),
    rdf(VPO,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object_Matrix),
    rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Robot_Matrix),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O1x))),atom_to_term(O1x,OX,_),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O1y))),atom_to_term(O1y,OY,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O2x))),atom_to_term(O2x,RX,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O2y))),atom_to_term(O2y,RY,_),
    rdf(Robot,'http://knowrob.org/kb/knowrob.owl#depthOfObject', literal(type(_,Pd))),atom_to_term(Pd,RD,_),
    rdf(Robot,'http://knowrob.org/kb/knowrob.owl#widthOfObject', literal(type(_,Pw))),atom_to_term(Pw,RW,_),
    Min_RY = RY - 0.5*RW,
    Min_RX = RX - 0.5*RD,
    Max_RY = RY + 0.5*RW,
    Max_RX = RX + 0.5*RD,
    rotation(RX,RY,OX,OY,-D,X1,Y1),
    once(
    (Min_RX =< X1, Max_RX >= X1 , Max_RY >= Y1, Min_RY =<Y1, RCCD='identical');
    ( check_behindRightArea(RX, RY, X1, Y1) , RCCD='behindRight');
    ( check_RightArea(RX, RY, X1, Y1) , RCCD='right');
    ( check_BehindArea(RX,RY, X1 ,Y1) , RCCD='behind');
    ( check_behindLeftArea(RX, RY, X1, Y1) , RCCD='behindLeft');
    ( check_LeftArea(RX, RY, X1, Y1) , RCCD='left');
    ( check_frontRightArea(RX, RY, X1, Y1) , RCCD='frontRight');
    ( check_frontLeftArea(RX, RY, X1, Y1) , RCCD='frontLeft');
    ( check_FrontArea(RX, RY, X1, Y1)   ,RCCD='front')


    ).



 comp_RCCD_P(Object, Robot, D,RCCD) :-
    rdfs_individual_of(Object,  knowrob:'Artifact'),
    rdfs_individual_of(Robot,  knowrob:'Artifact'),
    rdf(Robot, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
    latest_detection_of_instance(Object, VPO),
    latest_location_detection_of_instance(Robot,VPR),
    rdf(VPO,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object_Matrix),
    rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Robot_Matrix),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O1x))),atom_to_term(O1x,OX,_),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O1y))),atom_to_term(O1y,OY,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O2x))),atom_to_term(O2x,RX,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O2y))),atom_to_term(O2y,RY,_),
    rotation(RX,RY,OX,OY,-D,X1,Y1),
    rdf(Robot,'http://knowrob.org/kb/knowrob.owl#depthOfObject', literal(type(_,Pd))),atom_to_term(Pd,RD,_),
    rdf(Robot,'http://knowrob.org/kb/knowrob.owl#widthOfObject', literal(type(_,Pw))),atom_to_term(Pw,RW,_),
   Min_RY = RY - 0.5*RW,
   Min_RX = RX - 0.5*RD,
   Max_RY = RY + 0.5*RW,
   Max_RX = RX + 0.5*RD,
    once(
    (Min_RX =< X1, Max_RX >= X1 , Max_RY >= Y1, Min_RY =<Y1, RCCD='identical');
    (Min_RX =< X1, Max_RX >= X1 , Max_RY =< Y1, RCCD='front');

    (Min_RX >=X1 , Min_RY =<Y1, Max_RY >= Y1, RCCD='left');

    (Min_RX =< X1, Max_RX >=X1, Min_RY>= Y1 , RCCD='behind');
    (Max_RX =< X1, Max_RY >= Y1, Min_RY =< Y1 , RCCD='right');
    (Min_RX >=X1, Min_RY >=Y1 , RCCD='behindLeft');
     (Min_RX >=X1, Max_RY =< Y1 , RCCD='frontLeft');
    (Max_RX =< X1, Min_RY >= Y1 , RCCD='behindRight');
    (Max_RX =< X1, Max_RY =<Y1 , RCCD='frontRight')).


comp_RCCD_P2(Object, Robot, D,RCCD,Min_RY,Min_RX,Max_RY,Max_RX,X1,Y1) :-
    rdfs_individual_of(Object,  knowrob:'Artifact'),
    rdfs_individual_of(Robot,  knowrob:'Artifact'),
    rdf(Robot, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
    latest_detection_of_instance(Object, VPO),
    latest_location_detection_of_instance(Robot,VPR),
    rdf(VPO,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object_Matrix),
    rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Robot_Matrix),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O1x))),atom_to_term(O1x,OX,_),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O1y))),atom_to_term(O1y,OY,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O2x))),atom_to_term(O2x,RX,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O2y))),atom_to_term(O2y,RY,_),
    rotation(RX,RY,OX,OY,-D,X1,Y1),
    rdf(Robot,'http://knowrob.org/kb/knowrob.owl#depthOfObject', literal(type(_,Pd))),atom_to_term(Pd,RD,_),
    rdf(Robot,'http://knowrob.org/kb/knowrob.owl#widthOfObject', literal(type(_,Pw))),atom_to_term(Pw,RW,_),
   Min_RY = RY - 0.5*RW,
   Min_RX = RX - 0.5*RD,
   Max_RY = RY + 0.5*RW,
   Max_RX = RX + 0.5*RD,
    once(
    (Min_RX =< X1, Max_RX >= X1 , Max_RY >= Y1, Min_RY =<Y1, RCCD='identical');
    (Min_RX =< X1, Max_RX >= X1 , Max_RY =< Y1, RCCD='front');
    (Min_RX >=X1, Max_RY =< Y1 , RCCD='frontLeft');
    (Min_RX >=X1 , Min_RY =<Y1, Max_RY >= Y1, RCCD='left');
    (Min_RX >=X1, Min_RY >=Y1 , RCCD='behindLeft');
    (Min_RX =< X1, Max_RX >=X1, Min_RY>= Y1 , RCCD='behind');
    (Max_RX =< X1, Min_RY >= Y1 , RCCD='behindRight');
    (Max_RX =< X1, Max_RY >= Y1, Min_RY =< Y1 , RCCD='right');
    (Max_RX =< X1, Max_RY =<Y1 , RCCD='frontRight')).

find_frontPoint(CX, CY, TX, TY, Min_CX, CP) :- % find contact point cone & projection based
    getM(112.5,FM1),
    getFragment(CX,CY,FM1,F1),
    CP =FM1*Min_CX+F1.

find_rightPoint(CX, CY, TX, TY, Max_CY, CP) :- % find contact point cone & projection based
    getM(22.5,FM1),
    getFragment(CX,CY,FM1,F1),
    CP =(Max_CY - F1)/ FM1.

find_leftPoint(CX, CY, TX, TY, Max_CY, CP) :- % find contact point cone & projection based
    getM(157.5,FM1),
    getFragment(CX,CY,FM1,F1),
    CP =(Max_CY - F1)/ FM1.

find_behindPoint(CX, CY, TX, TY, Max_CX, CP) :- % find contact point cone & projection based
    getM(292.5,FM1),
    getFragment(CX,CY,FM1,F1),
    CP =FM1*Max_CX+F1.

comp_RCCD_Hybrid(Object, Robot, RCCD) :-
   rdfs_individual_of(Object,  knowrob:'SpatialThing'),
    rdfs_individual_of(Robot,  knowrob:'SpatialThing'),
    rdf(Robot, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
    latest_detection_of_instance(Object, VPO),
    latest_location_detection_of_instance(Robot,VPR),


    rdf(VPO,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object_Matrix),
    rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Robot_Matrix),
    rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventHasValue',literal(type(_,A))),atom_to_term(A,D,_),

    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O1x))),atom_to_term(O1x,OX,_),
    rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O1y))),atom_to_term(O1y,OY,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O2x))),atom_to_term(O2x,RX,_),
    rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O2y))),atom_to_term(O2y,RY,_),



    rotation(RX,RY,OX,OY,-D,X1,Y1),
    rdf(Robot,'http://knowrob.org/kb/knowrob.owl#depthOfObject', literal(type(_,Pd))),atom_to_term(Pd,RD,_),
    rdf(Robot,'http://knowrob.org/kb/knowrob.owl#widthOfObject', literal(type(_,Pw))),atom_to_term(Pw,RW,_),
    Min_RY = RY - 0.5*RW,
    Min_RX = RX - 0.5*RD,
    Max_RY = RY + 0.5*RW,
    Max_RX = RX + 0.5*RD,
    once(
    (Min_RX =< X1, Max_RX >= X1 , Max_RY >= Y1, Min_RY =<Y1, RCCD='identical');

    %check hybrid front
    ( find_frontPoint(RX, RY, X1, Y1, Min_RX, CP),    %find contact point
    once(
    (Y1 =< CP,Min_RX =< X1, Max_RX >= X1 , Max_RY =< Y1, RCCD='front'); %projection based
    (Y1 > CP, check_FrontArea(RX, RY, X1, Y1)   ,RCCD='front') %cone based
    ));
    %check front end

    %check hybrid right
    ( find_rightPoint(RX, RY, X1, Y1, Max_RY, CP),    %find contact point
    once(
    (X1 =< CP,Min_RX =< X1, Max_RX =< X1, Max_RY >= Y1, Min_RY =< Y1, RCCD='right'); %projection based
    (X1 > CP, check_RightArea(RX, RY, X1, Y1)   ,RCCD='right') %cone based
    ));
    %check right end

    %check hybrid left
    ( find_leftPoint(RX, RY, X1, Y1, Max_RY, CP),    %find contact point
    once(
    (X1 >= CP,Min_RX =< X1, Max_RX =< X1, Max_RY >= Y1, Min_RY =< Y1, RCCD='left'); %projection based
    (X1 < CP, check_LeftArea(RX, RY, X1, Y1)   ,RCCD='left') %cone based
    ));
    %check left end

    %check hybrid behind
    ( find_behindPoint(RX, RY, X1, Y1, Max_RX, CP),    %find contact point
    once(
    (Y1 >= CP,Min_RX =< X1, Max_RX >=X1, Min_RY>= Y1, RCCD='behind'); %projection based
    (Y1 < CP, check_BehindArea(RX, RY, X1, Y1)   ,RCCD='behind') %cone based
    ));
    %check behind end

    (Max_RX =< X1, Min_RY >= Y1 ,check_behindRightArea(RX, RY, X1, Y1), RCCD='behindRight');
    (Min_RX >=X1, Max_RY =< Y1 ,check_frontLeftArea(RX, RY, X1, Y1),  RCCD='frontLeft');
    (Min_RX >=X1, Min_RY >=Y1 , check_behindLeftArea(RX, RY, X1, Y1), RCCD='behindLeft');
    (Max_RX =< X1, Max_RY =<Y1 , check_frontRightArea(RX, RY, X1, Y1), RCCD='frontRight')
    ).








  comp_identical(Object1, Object2,B) :-
    rdfs_individual_of(Object1,  knowrob:'Artifact'),
    rdfs_individual_of(Object2,  knowrob:'Artifact'),
    (latest_detection_of_instance(Object1, VPO1);latest_location_detection_of_instance(Object1,VPO1)),
    (latest_location_detection_of_instance(Object2,VPO2);latest_detection_of_instance(Object2, VPO2)),
    rdf(VPO1,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object1_Matrix),
    rdf(VPO2,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object2_Matrix),
    rdf(Object1_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O1x))),atom_to_term(O1x,O1X,_),
    rdf(Object1_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O1y))),atom_to_term(O1y,O1Y,_),
    rdf(Object2_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O2x))),atom_to_term(O2x,O2X,_),
    rdf(Object2_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O2y))),atom_to_term(O2y,O2Y,_),
    DX= O2X - O1X,
    DY= O2Y - O2Y,
    DX=:=0, DY=:= 0,
    Object1 \= Object2.



in_FrontArea(X,Y,Place) :-
	rdf(Place, 'http://www.arbi.com/ontologies/arbi.owl#frontAreaOfRoom', FrontArea),
	rdf(FrontArea, 'http://knowrob.org/kb/knowrob.owl#center', Point),
	rdf(Point,'http://knowrob.org/kb/knowrob.owl#xCoord', literal(type(_,Px))), atom_to_term(Px,PX,_),
	rdf(Point,'http://knowrob.org/kb/knowrob.owl#yCoord', literal(type(_,Py))), atom_to_term(Py,PY,_),
	rdf(FrontArea,'http://knowrob.org/kb/knowrob.owl#depthOfObject', literal(type(_,Pd))),atom_to_term(Pd,PD,_),
	rdf(FrontArea,'http://knowrob.org/kb/knowrob.owl#widthOfObject', literal(type(_,Pw))),atom_to_term(Pw,PW,_),

	Min_placeY = PY - 0.5*PW,
	Min_placeX = PX - 0.5*PD,
	Max_placeY = PY + 0.5*PW,
	Max_placeX = PX + 0.5*PD,
	Min_placeX =< X,
	Min_placeY =< Y,
	Max_placeX >= X,
	Max_placeY >= Y.

in_InsideArea(X,Y,Place) :-
	rdf(Place, 'http://knowrob.org/kb/knowrob.owl#center', Point),
	rdf(Point,'http://knowrob.org/kb/knowrob.owl#xCoord', literal(type(_,Px))), atom_to_term(Px,PX,_),
	rdf(Point,'http://knowrob.org/kb/knowrob.owl#yCoord', literal(type(_,Py))), atom_to_term(Py,PY,_),
	rdf(Place,'http://knowrob.org/kb/knowrob.owl#depthOfObject', literal(type(_,Pd))),atom_to_term(Pd,PD,_),
	rdf(Place,'http://knowrob.org/kb/knowrob.owl#widthOfObject', literal(type(_,Pw))),atom_to_term(Pw,PW,_),
	Min_placeY = PY - 0.5*PW,
	Min_placeX = PX - 0.5*PD,
	Max_placeY = PY + 0.5*PW,
	Max_placeX = PX + 0.5*PD,
	Min_placeX =< X,
	Max_placeX >= X,
	Min_placeY =< Y,
	Max_placeY >= Y.



insideOfRoom(Object,Room) :-
	comp_insideOfRoom(Object,Room).

locatedInRoom(Object,Room) :-
     get_timepoint(T),
     holds(locatedInRoom(Object, Room), T).

holds(locatedInRoom(Object, Room), T) :-
      rdfs_individual_of(Object,  knowrob:'SpatialThing'),
      (latest_detection_of_instance(Object, VP);latest_location_detection_of_instance(Object,VP)),
	rdfs_individual_of(Room, 'http://knowrob.org/kb/knowrob.owl#SpaceInAHOC'),
	rdf(VP,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object_Matrix),
	rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,Rx))),atom_to_term(Rx,RX,_),
	rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,Ry))),atom_to_term(Ry,RY,_),
	in_InsideArea(RX,RY,Room).


personLocatedInRoom(Person, Room) :-
     get_timepoint(T),
     once(
     	(rdf(Person, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Person'),
     	holds(locatedInRoom(Person, Room), T));
     
     	(rdf(Schedule, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://www.arbi.com/ontologies/arbi.owl#Schedule'),
     	rdf(Schedule,'http://www.arbi.com/ontologies/arbi.owl#hasAttendee', Attendee),
     	=(Person, Attendee),
          %rdf(Schedule, 'http://knowrob.org/kb/knowrob.owl#startTime', ST),
 	    %rdf(Schedule, 'http://knowrob.org/kb/knowrob.owl#endTime', ET),
	    %rdf_assert(ST, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#TimePoint', query),
 	    %rdf_assert(ET, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#TimePoint', query),
 	    %comp_after(SE ,  T),
  	    %comp_after(T ,  ET),
     	rdf(Schedule, 'http://www.arbi.com/ontologies/arbi.owl#scheduleOccursAt', Room)
     
     )
     ).


latestObjectXpoint(Xpoint, Object) :-
	rdfs_individual_of(Object,  knowrob:'SpatialThing'),
      (latest_detection_of_instance(Object, VP);latest_location_detection_of_instance(Object,VP)),
	rdf(VP,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object_Matrix),
	rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,Rx))),atom_to_term(Rx,Xpoint,_).

latestObjectYpoint(Ypoint, Object) :-
	rdfs_individual_of(Object,  knowrob:'SpatialThing'),
      (latest_detection_of_instance(Object, VP);latest_location_detection_of_instance(Object,VP)),
	rdf(VP,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object_Matrix),
	rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,Ry))),atom_to_term(Ry,Ypoint,_).

latestObjectZpoint(Zpoint, Object) :-
	rdfs_individual_of(Object,  knowrob:'SpatialThing'),
      (latest_detection_of_instance(Object, VP);latest_location_detection_of_instance(Object,VP)),
	rdf(VP,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Object_Matrix),
	rdf(Object_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,Ry))),atom_to_term(Ry,Zpoint,_).











insideAreaOfRoom(Robot,Room) :-
	comp_insideAreaOfRoom(Robot,Room).

comp_insideAreaOfRoom(Robot,Room) :-
     get_timepoint(T),
     holds(comp_insideAreaOfRoom(Robot, Room), T).

holds(comp_insideAreaOfRoom(Robot, Room), T) :-
      rdf(Robot, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
      latest_location_detection_of_instance(Robot,VPR),
	rdf(Room, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type','http://knowrob.org/kb/knowrob.owl#RoomInAConstruction'),
	rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Robot_Matrix),
	rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,Rx))),atom_to_term(Rx,RX,_),
	rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,Ry))),atom_to_term(Ry,RY,_),
	in_InsideArea(RX,RY,Room).

inFrontAreaOfRoom(Robot,Room) :-
	comp_inFrontAreaOfRoom(Robot,Room).
comp_inFrontAreaOfRoom(Robot,Room) :-
     get_timepoint(T),
     holds(comp_inFrontAreaOfRoom(Robot, Room), T).

holds(comp_inFrontAreaOfRoom(Robot, Room), T) :-
      rdf(Robot, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
      latest_location_detection_of_instance(Robot,VPR),
      rdf(Room, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type','http://knowrob.org/kb/knowrob.owl#RoomInAConstruction'),
	rdf(VPR,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Robot_Matrix),
	rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,Rx))),atom_to_term(Rx,RX,_),
	rdf(Robot_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,Ry))),atom_to_term(Ry,RY,_),
	in_FrontArea(RX,RY,Room).





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

 latest_location_detection_of_instance(Object, LatestDetection) :-
  ((rdf_has(Object, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Object,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              rdfs_individual_of(D_i,  arbi:'LocationPerception'),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),

    % compute the homography for the newest perception
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).
    
    
    

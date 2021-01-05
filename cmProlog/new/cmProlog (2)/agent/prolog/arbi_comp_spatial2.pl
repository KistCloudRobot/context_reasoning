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
    inFrontAreaOfRoom/2,
    insideAreaOfRoom/2,
    comp_inFrontAreaOfRoom/2,
    comp_insideAreaOfRoom/2,
    latest_location_detection_of_instance/2
   
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_objects')).
:- use_module(library('knowrob_owl')).


:- rdf_db:rdf_register_ns(knowrob,      'http://knowrob.org/kb/knowrob.owl#',      [keep(true)]).


:- meta_predicate holds(0, ?, ?).
:- discontiguous holds/2.
:-  rdf_meta
    holds(:, r),
    latest_detection_of_instance(r,r).


in_FrontArea(X,Y,Place) :-
	rdf(Place, 'http://www.arbi.com/arbi#entranceZoonOfRoom', FrontArea),
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
                              rdfs_individual_of(D_i,  knowrob:'MentalEvent'),
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
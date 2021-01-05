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
:- module(comp_spatial,
    [
      holds/2,
      in_ContGeneric/2,
      toTheRightOf/5,
      toTheRightOf/2,
      comp_toTheRightOf/2,
      toTheLeftOf/2,
      toTheLeftOf/5,
      comp_toTheLeftOf/2,
      comp_toTheSideOf/2,
      behindOf/2,
      behindOf/5,
      comp_behindOf/2,
      comp_inFrontOf/2,
      inFrontOf/5,
      inFrontOf/2,
      comp_inCenterOf/2,
      comp_below_of/2,
      belowOf/2,
      comp_above_of/2,
      aboveOf/2,
      comp_center/2,
      objectAtPoint2D/2,
      latest_detection_of_instance/2,
      latest_location_detection_of_instance/2,
      objectAtPoint2D/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_objects')).
:- use_module(library('knowrob_owl')).


:- rdf_db:rdf_register_ns(knowrob,      'http://knowrob.org/kb/knowrob.owl#',      [keep(true)]).
:- rdf_db:rdf_register_ns(comp_spatial, 'http://knowrob.org/kb/comp_spatial.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(arbi, 'http://www.arbi.com/ontologies/arbi.owl#',     [keep(true)]).


% define holds as meta-predicate and allow the definitions
% to be in different parts of the source file
:- meta_predicate holds(0, ?, ?).
:- discontiguous holds/2.

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
    holds(:, r),
    in_ContGeneric(r, r),
    adjacent_Objects(r, r),
    comp_below_of(r,r),
    comp_above_of(r,r),
    comp_behindOf(r,r),
    behindOf(r,r),
    behindOf(r,r,r,r,r),
    inFrontOf(r,r,r,r,r),
    inFrontOf(r,r),
    comp_toTheSideOf(r, r),
    toTheRightOf(r,r,r,r,r),
    toTheRightOf(r,r),
    comp_toTheRightOf(r, r),
    comp_toTheLeftOf(r, r),
    toTheLeftOf(r,r),
    toTheLeftOf(r,r,r,r,r),
    comp_inFrontOf(r, r),
    comp_inCenterOf(r, r),
    comp_center(r, r).




%% on_Physical(?Top, ?Bottom) is nondet.
%
% Check if Top is in the area of and above Bottom.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Top Identifier of the upper Object
% @param Bottom Identifier of the lower Object
%




%% comp_above_of(?Top, ?Bottom) is nondet.
%
% Check if Top is in the area of and above Bottom.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Top Identifier of the upper Object
% @param Bottom Identifier of the lower Object
%

aboveOf(Top, Bottom) :-
    distinct(comp_above_of(Top, Bottom)).
    
comp_above_of(Top, Bottom) :-
    get_timepoint(T),
    holds(comp_above_of(Top, Bottom),T).
    
%% holds(+BelowOf:compound, +T) is nondet.
%
% Usage: holds(comp_above_of(?Top, ?Bottom), +T)
%
% Check if Bottom has been in the area of and below Top at time point T.
%
% Currently does not take the orientation into account, only the position and dimension.
%
% @param Top    Identifier of the upper Object
% @param Bottom Identifier of the lower Object
% @param T      TimePoint or Event for which the relations is supposed to hold
%
holds(comp_above_of(Top, Bottom),T) :-

    % TODO: adapt this to take rotations and object dimensions into account
    rdfs_individual_of(Top,  knowrob:'Artifact'),
    rdfs_individual_of(Bottom,  knowrob:'Artifact'),    
    (latest_detection_of_instance(Top, VPF)),
    (latest_detection_of_instance(Bottom, VPB)),

    rdf_triple(knowrob:eventOccursAt, VPF, TopMatrix),
    rdf_triple(knowrob:eventOccursAt, VPB, BottomMatrix),

    rdf_triple(knowrob:m23, TopMatrix, literal(type(_,ICz))),atom_to_term(ICz,FZ,_),

    % read the center coordinates of the right entity

    rdf_triple(knowrob:m23, BottomMatrix, literal(type(_,OCz))),atom_to_term(OCz,BZ,_),

   % =<( abs( FX - BX), 3.5),  % less than 350cm x diff
   %  =<( abs( FY - BY), 1.0),  % less than 150cm y diff
   %=<( abs( FZ - BZ), 3.5),  % less than 350cm z diff
    =<( BZ, FZ ),      % front obj has a higher x coord
    =<(0.05,FZ-BZ),
    Bottom \= Top.



%% comp_below_of(?Bottom, ?Top) is nondet.
%
% Check if Top is in the area of and above Bottom.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Bottom Identifier of the lower Object
% @param Top Identifier of the upper Object
%

belowOf(Bottom, Top) :-
    distinct(comp_below_of(Bottom, Top)).
    
comp_below_of(Bottom, Top) :-
    get_timepoint(T),
    holds(comp_below_of(Bottom, Top),T).

holds(comp_below_of(Bottom, Top), T) :-

    % TODO: adapt this to take rotations and object dimensions into account
    rdfs_individual_of(Bottom,  knowrob:'Artifact'),
    rdfs_individual_of(Top,  knowrob:'Artifact'),    
    latest_detection_of_instance(Bottom, VPF),
    latest_detection_of_instance(Top, VPB),

    rdf_triple(knowrob:eventOccursAt, VPF, BottomMatrix),
    rdf_triple(knowrob:eventOccursAt, VPB, TopMatrix),
    rdf_triple(knowrob:m23, BottomMatrix, literal(type(_,ICz))),atom_to_term(ICz,FZ,_),

    % read the center coordinates of the right entity
    rdf_triple(knowrob:m23, TopMatrix, literal(type(_,OCz))),atom_to_term(OCz,BZ,_),

   % =<( abs( FX - BX), 3.5),  % less than 350cm x diff
   %  =<( abs( FY - BY), 1.0),  % less than 150cm y diff
   %=<( abs( FZ - BZ), 3.5),  % less than 350cm z diff
    =<( FZ, BZ ),      % front obj has a higher x coord
    =<(0.05, BZ-FZ),
    Bottom \= Top.

    
toTheLeftOf(Left, Right) :-
	comp_toTheLeftOf(Left, Right).

%% comp_toTheLeftOf(?Left, ?Right) is nondet.
%
% Check if Left is to the left of Right.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Left Identifier of the left Object
% @param Right Identifier of the right Object
%
comp_toTheLeftOf(Left, Right) :-
    rdfs_individual_of(Left,  knowrob:'Artifact'),
    rdfs_individual_of(Right,  knowrob:'Artifact'),
    rdf(Right, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
    latest_detection_of_instance(Left, VPL),
    latest_location_detection_of_instance(Right,VPR),
    %print(Left),
    %print(Right).
  
    rdf_triple(knowrob:eventOccursAt, VPL, LeftMatrix),
    rdf_triple(knowrob:eventOccursAt, VPR, RightMatrix),

    % read the center coordinates of the left entity
    rdf_triple(knowrob:m03, LeftMatrix, literal(type(_,LCx))),atom_to_term(LCx,LX,_),
    rdf_triple(knowrob:m13, LeftMatrix, literal(type(_,LCy))),atom_to_term(LCy,LY,_),
  %  rdf_triple(knowrob:m23, LeftMatrix, literal(type(_,LCz))),atom_to_term(LCz,LZ,_),

    % read the center coordinates of the right entity
    rdf_triple(knowrob:m03, RightMatrix, literal(type(_,RCx))),atom_to_term(RCx,RX,_),
    rdf_triple(knowrob:m13, RightMatrix, literal(type(_,RCy))),atom_to_term(RCy,RY,_),
 %   rdf_triple(knowrob:m23, RightMatrix, literal(type(_,RCz))),atom_to_term(RCz,RZ,_),

    =<( abs( LX - RX), 1.5),  % less than 30cm y diff
  %  =<( abs( LY - RY), 4.5),
   =<( RY, LY ),              % right obj has a smaller y coord than the left one (on the table)
  %   =<( abs( LZ - RZ), 0.30),  % less than 30cm height diff
    Left \= Right.


toTheLeftOf(Left, Right, P1, P2, P3) :-
	comp_toTheLeftOf(Left, Right, P1, P2, P3).

%% comp_toTheLeftOf(?Left, ?Right) is nondet.
%
% Check if Left is to the left of Right.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Left Identifier of the left Object
% @param Right Identifier of the right Object
%
comp_toTheLeftOf(Left, Right, P1, P2, P3) :-
    rdfs_individual_of(Left,  knowrob:'Artifact'),
    rdfs_individual_of(Right,  knowrob:'Artifact'),
   % rdf(Right, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
    (latest_detection_of_instance(Left, VPL);latest_location_detection_of_instance(Left,VPL)),
    (latest_location_detection_of_instance(Right,VPR);latest_detection_of_instance(Right, VPR)),
  %  latest_detection_of_instance(Left, VPL),
  %  latest_location_detection_of_instance(Right,VPR),
  
  
    rdf_triple(knowrob:eventOccursAt, VPL, LeftMatrix),
    rdf_triple(knowrob:eventOccursAt, VPR, RightMatrix),

    % read the center coordinates of the left entity
    rdf_triple(knowrob:m03, LeftMatrix, literal(type(_,LCx))),atom_to_term(LCx,LX,_),
    rdf_triple(knowrob:m13, LeftMatrix, literal(type(_,LCy))),atom_to_term(LCy,LY,_),
    rdf_triple(knowrob:m23, LeftMatrix, literal(type(_,LCz))),atom_to_term(LCz,LZ,_),

    % read the center coordinates of the right entity
    rdf_triple(knowrob:m03, RightMatrix, literal(type(_,RCx))),atom_to_term(RCx,RX,_),
    rdf_triple(knowrob:m13, RightMatrix, literal(type(_,RCy))),atom_to_term(RCy,RY,_),
    rdf_triple(knowrob:m23, RightMatrix, literal(type(_,RCz))),atom_to_term(RCz,RZ,_),

    =<( abs( LX - RX), P1),  % less than 30cm y diff
    =<( abs( LY - RY), P2),
    =<( RY, LY ),              % right obj has a smaller y coord than the left one (on the table)
    =<( abs( LZ - RZ), P3),  % less than 30cm height diff
    Left \= Right.












%% comp_toTheRightOf(?Right,?Left) is nondet.
%
% Check if Right is to the right of Left.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Right Identifier of the right Object
% @param Left Identifier of the left Object
% @see comp_toTheLeftOf
%

toTheRightOf(Right, Left) :-
	comp_toTheRightOf(Right, Left).
comp_toTheRightOf(Right, Left) :-
    get_timepoint(T),
    holds(comp_toTheRightOf(Right, Left), T).


%% holds(+ToTheRightOf:compound, +T) is nondet.
%
% Usage: holds(comp_toTheRightOf(?Right,?Left), +T)
%
% Check if Right is to the right of Left.
%
% @param Right Identifier of the right Object
% @param Left Identifier of the left Object
% @param T      TimePoint or Event for which the relations is supposed to hold
% @see comp_toTheLeftOf
%
holds(comp_toTheRightOf(Right, Left), T) :-
    rdfs_individual_of(Left,  knowrob:'Artifact'),
    rdfs_individual_of(Right,  knowrob:'Artifact'),
    rdf(Left, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
    latest_detection_of_instance(Right, VPR),
    latest_location_detection_of_instance(Left,VPL),
  %  latest_detection_of_instance(Left, VPL),
  %  latest_location_detection_of_instance(Right,VPR),
  
  
    rdf_triple(knowrob:eventOccursAt, VPL, LeftMatrix),
    rdf_triple(knowrob:eventOccursAt, VPR, RightMatrix),

    % read the center coordinates of the left entity
    rdf_triple(knowrob:m03, LeftMatrix, literal(type(_,LCx))),atom_to_term(LCx,LX,_),
    rdf_triple(knowrob:m13, LeftMatrix, literal(type(_,LCy))),atom_to_term(LCy,LY,_),
  %  rdf_triple(knowrob:m23, LeftMatrix, literal(type(_,LCz))),atom_to_term(LCz,LZ,_),

    % read the center coordinates of the right entity
    rdf_triple(knowrob:m03, RightMatrix, literal(type(_,RCx))),atom_to_term(RCx,RX,_),
    rdf_triple(knowrob:m13, RightMatrix, literal(type(_,RCy))),atom_to_term(RCy,RY,_),
   % rdf_triple(knowrob:m23, RightMatrix, literal(type(_,RCz))),atom_to_term(RCz,RZ,_),
    
   % =<( abs( RY- LY), 4.5), 
    =<( abs( LX - RX), 1.5),  % less than 30cm y diff
    =<( RY, LY ),              % right obj has a smaller y coord than the left one (on the table)
    %=<( abs( LZ - RZ), 0.30),  % less than 30cm height diff
    Left \= Right.

toTheRightOf(Right,Left, P1, P2, P3) :-
	comp_toTheRightOf(Right, Left, P1, P2, P3).
comp_toTheRightOf(Right, Left, P1, P2, P3) :-
    get_timepoint(T),
    holds(comp_toTheRightOf(Right, Left, P1, P2, P3), T).


%% holds(+ToTheRightOf:compound, +T) is nondet.
%
% Usage: holds(comp_toTheRightOf(?Right,?Left), +T)
%
% Check if Right is to the right of Left.
%
% @param Right Identifier of the right Object
% @param Left Identifier of the left Object
% @param T      TimePoint or Event for which the relations is supposed to hold
% @see comp_toTheLeftOf
%
holds(comp_toTheRightOf(Right, Left, P1, P2, P3), T) :-
     rdfs_individual_of(Left,  knowrob:'Artifact'),
    rdfs_individual_of(Right,  knowrob:'Artifact'),
   % rdf(Right, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#Robot'),
    (latest_detection_of_instance(Right, VPR);latest_location_detection_of_instance(Right,VPR)),
    (latest_location_detection_of_instance(Left,VPL);latest_detection_of_instance(Left, VPL)),
  %  latest_detection_of_instance(Left, VPL),
  %  latest_location_detection_of_instance(Right,VPR),
  
  
    rdf_triple(knowrob:eventOccursAt, VPL, LeftMatrix),
    rdf_triple(knowrob:eventOccursAt, VPR, RightMatrix),

    % read the center coordinates of the left entity
    rdf_triple(knowrob:m03, LeftMatrix, literal(type(_,LCx))),atom_to_term(LCx,LX,_),
    rdf_triple(knowrob:m13, LeftMatrix, literal(type(_,LCy))),atom_to_term(LCy,LY,_),
    rdf_triple(knowrob:m23, LeftMatrix, literal(type(_,LCz))),atom_to_term(LCz,LZ,_),

    % read the center coordinates of the right entity
    rdf_triple(knowrob:m03, RightMatrix, literal(type(_,RCx))),atom_to_term(RCx,RX,_),
    rdf_triple(knowrob:m13, RightMatrix, literal(type(_,RCy))),atom_to_term(RCy,RY,_),
    rdf_triple(knowrob:m23, RightMatrix, literal(type(_,RCz))),atom_to_term(RCz,RZ,_),
    
    =<( abs( RY- LY), P2),  % less than 30cm y diff
    =<( abs( LX - RX), P1),  
    =<( RY, LY ),              % right obj has a smaller y coord than the left one (on the table)
    =<( abs( LZ - RZ), P3),  % less than 30cm height diff
    Left \= Right.

%% comp_toTheSideOf(?A, ?B) is nondet.
%
% Check if A is either to the left or the rigth of B.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param A Identifier of Object A
% @param B Identifier of Object B
% @see comp_toTheLeftOf
% @see comp_toTheRightOf
%
comp_toTheSideOf(A, B) :-
    get_timepoint(T),
    holds(comp_toTheSideOf(A, B), T).

%% holds(+ToTheLeftOf:compound, +T) is nondet.
%
% Usage: holds(comp_toTheSideOf(?A, ?B), +T) is nondet.
%
% Check if A is either to the left or the right of B.
%
% @param A Identifier of Object A
% @param B Identifier of Object B
% @param T TimePoint or Event for which the relations is supposed to hold
% @see comp_toTheLeftOf
% @see comp_toTheRightOf
%
holds(comp_toTheSideOf(A, B), T) :-
    holds(comp_toTheRightOf(A, B), T);
    holds(comp_toTheLeftOf(A, B), T).


inFrontOf(Front, Back,P1,P2,P3) :-
	comp_inFrontOf(Front, Back,P1,P2,P3).
comp_inFrontOf(Front, Back,P1,P2,P3) :-
    get_timepoint(T),
    holds(comp_inFrontOf(Front, Back,P1,P2,P3), T).

holds(comp_inFrontOf(Front, Back,P1,P2,P3), T) :-

    % TODO: adapt this to take rotations and object dimensions into account
    rdfs_individual_of(Front,  knowrob:'Artifact'),
    rdfs_individual_of(Back,  knowrob:'Artifact'),    
    (latest_detection_of_instance(Front, VPF);latest_location_detection_of_instance(Front,VPF)),
    (latest_location_detection_of_instance(Back,VPB);latest_detection_of_instance(Back, VPB)),

    rdf_triple(knowrob:eventOccursAt, VPF, FrontMatrix),
    rdf_triple(knowrob:eventOccursAt, VPB, BackMatrix),

    rdf_triple(knowrob:m03, FrontMatrix, literal(type(_,ICx))),atom_to_term(ICx,FX,_),
    rdf_triple(knowrob:m13, FrontMatrix, literal(type(_,ICy))),atom_to_term(ICy,FY,_),
  %  rdf_triple(knowrob:m23, FrontMatrix, literal(type(_,ICz))),atom_to_term(ICz,FZ,_),

    % read the center coordinates of the right entity
    rdf_triple(knowrob:m03, BackMatrix, literal(type(_,OCx))),atom_to_term(OCx,BX,_),
    rdf_triple(knowrob:m13, BackMatrix, literal(type(_,OCy))),atom_to_term(OCy,BY,_),
   % rdf_triple(knowrob:m23, BackMatrix, literal(type(_,OCz))),atom_to_term(OCz,BZ,_),

    =<( abs( FX - BX), P1),  % less than 20cm x diff
    =<( abs( FY - BY), P2),  % less than 20cm y diff
    %=<( abs( FZ - BZ), P3),  % less than 20cm z diff
    =<( BX, FX ),      % front obj has a higher x coord
    Front \= Back.
   
    
inFrontOf(Front, Back) :-
     comp_inFrontOf(Front, Back).
comp_inFrontOf(Front, Back) :-
    get_timepoint(T),
    holds(comp_inFrontOf(Front, Back), T).

holds(comp_inFrontOf(Front, Back), T) :-

    % TODO: adapt this to take rotations and object dimensions into account
    rdfs_individual_of(Front,  knowrob:'Artifact'),
    rdfs_individual_of(Back,  knowrob:'Artifact'),    
    (latest_detection_of_instance(Front, VPF);latest_location_detection_of_instance(Front,VPF)),
    (latest_location_detection_of_instance(Back,VPB);latest_detection_of_instance(Back, VPB)),

    rdf(VPF, knowrob:eventOccursAt, FrontMatrix),
    rdf(VPB, knowrob:eventOccursAt, BackMatrix),

    rdf(FrontMatrix, knowrob:m03, literal(type(_,ICx))),atom_to_term(ICx,FX,_),
    rdf(FrontMatrix, knowrob:m13, literal(type(_,ICy))),atom_to_term(ICy,FY,_),
  %  rdf_triple(knowrob:m23, FrontMatrix, literal(type(_,ICz))),atom_to_term(ICz,FZ,_),

    % read the center coordinates of the right entity
    rdf(BackMatrix, knowrob:m03, literal(type(_,OCx))),atom_to_term(OCx,BX,_),
    rdf(BackMatrix, knowrob:m13, literal(type(_,OCy))),atom_to_term(OCy,BY,_),
   % rdf_triple(knowrob:m23, BackMatrix, literal(type(_,OCz))),atom_to_term(OCz,BZ,_),

   % =<( abs( FX - BX), 3.5),  % less than 350cm x diff
   %  =<( abs( FY - BY), 1.0),  % less than 150cm y diff
   %=<( abs( FZ - BZ), 3.5),  % less than 350cm z diff
    =<( BX, FX ),      % front obj has a higher x coord
    Front \= Back.
    
    
    
    
    


%% comp_inFrontOf(?Front, ?Back) is nondet.
%
% Check if Front is in front of Back. Currently does not take the orientation
% into account, only the position and dimension.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Front Identifier of the front Object
% @param Back Identifier of the back Object
%
behindOf(Back,Front,P1,P2,P3) :-
    comp_behindOf(Back, Front, P1, P2, P3).
comp_behindOf(Back, Front, P1, P2, P3) :-
    get_timepoint(T),
    holds(comp_behindOf(Back, Front, P1, P2, P3), T).


holds(comp_behindOf(Back, Front, P1, P2, P3), T) :-
    holds(comp_inFrontOf(Front, Back, P1, P2, P3), T).	

behindOf(Back,Front) :-
    comp_behindOf(Back, Front).
comp_behindOf(Back, Front) :-
    get_timepoint(T),
    holds(comp_behindOf(Back, Front), T).


holds(comp_behindOf(Back, Front), T) :-
    holds(comp_inFrontOf(Front, Back), T).


%% comp_inCenterOf(?Inner, ?Outer) is nondet.
%
% Check if Inner is in the center of OuterObj. Currently does not take the orientation
% into account, only the position and dimension.
%
% Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param Inner Identifier of the inner Object
% @param Outer Identifier of the outer Object
%
comp_inCenterOf(Inner, Outer) :-
    get_timepoint(T),
    holds(comp_inCenterOf(Inner, Outer), T).

%% holds(+InCenterOf:compound, +T) is nondet.
%
% Usage: holds(comp_inCenterOf(?Inner, ?Outer), +T)
%
% Check if Inner is in the center of OuterObj. Currently does not take the orientation
% into account, only the position and dimension.
%
% @param Inner Identifier of the inner Object
% @param Outer Identifier of the outer Object
% @param T TimePoint or Event for which the relations is supposed to hold
%
holds(comp_inCenterOf(Inner, Outer), T) :-

    object_detection(Inner, T, VPI),
    object_detection(Outer, T, VPO),

    rdf_triple(knowrob:eventOccursAt, VPI, InnerMatrix),
    rdf_triple(knowrob:eventOccursAt, VPO, OuterMatrix),

    % read the center coordinates of the left entity
    rdf_triple(knowrob:m03, InnerMatrix, literal(type(_,ICx))),atom_to_term(ICx,IX,_),
    rdf_triple(knowrob:m13, InnerMatrix, literal(type(_,ICy))),atom_to_term(ICy,IY,_),
    rdf_triple(knowrob:m23, InnerMatrix, literal(type(_,ICz))),atom_to_term(ICz,IZ,_),

    % read the center coordinates of the right entity
    rdf_triple(knowrob:m03, OuterMatrix, literal(type(_,OCx))),atom_to_term(OCx,OX,_),
    rdf_triple(knowrob:m13, OuterMatrix, literal(type(_,OCy))),atom_to_term(OCy,OY,_),
    rdf_triple(knowrob:m23, OuterMatrix, literal(type(_,OCz))),atom_to_term(OCz,OZ,_),

    =<( abs( IX - OX), 0.20),  % less than 20cm x diff
    =<( abs( IY - OY), 0.20),  % less than 20cm y diff
    =<( abs( IZ - OZ), 0.20),  % less than 20cm z diff
    Inner \= Outer.


%% in_ContGeneric(?InnerObj, ?OuterObj) is nondet.
%
% Check if InnerObj is contained by OuterObj. Currently does not take the orientation
% into account, only the position and dimension.
%
                                % Implemented as a wrapper predicate around holds(...) that computes the relation for the
% current point in time
%
% @param InnerObj Identifier of the inner Object
% @param OuterObj Identifier of the outer Object
%
in_ContGeneric(InnerObj, OuterObj) :-
    get_timepoint(T),
    holds(in_ContGeneric(InnerObj, OuterObj), T).



%% holds(+InContained:compound, +T) is nondet.
%
% Usage: holds(in_ContGeneric(?InnerObj, ?OuterObj), +T)
%
% Check if Inner is in the center of OuterObj. Currently does not take the orientation
% into account, only the position and dimension.
%
% @param InnerObj Identifier of the inner Object
% @param OuterObj Identifier of the outer Object
% @param T TimePoint or Event for which the relations is supposed to hold
%
holds(in_ContGeneric(InnerObj, OuterObj), T) :-

    rdfs_individual_of(InnerObj,  knowrob:'SpatialThing-Localized'),
    rdfs_individual_of(OuterObj,  knowrob:'SpatialThing-Localized'),
    % not(rdfs_individual_of(OuterObj,  knowrob:'DisplayShelf')),
    latest_detection_of_instance(InnerObj, VPI),
    latest_detection_of_instance(OuterObj, VPO),
    rdf(VPI,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Inner_Matrix),
    rdf(VPO,'http://knowrob.org/kb/knowrob.owl#eventOccursAt', Outer_Matrix),
    rdf(Inner_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O1x))),atom_to_term(O1x,IX,_),
    rdf(Inner_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O1y))),atom_to_term(O1y,IY,_),
    rdf(Inner_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,O1z))),atom_to_term(O1z,IZ,_),
    rdf(Outer_Matrix,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(_,O2x))),atom_to_term(O2x,OX,_),
    rdf(Outer_Matrix,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(_,O2y))),atom_to_term(O2y,OY,_),
    rdf(Outer_Matrix,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(_,O2z))),atom_to_term(O2z,OZ,_),
    
    rdf(InnerObj,'http://knowrob.org/kb/knowrob.owl#depthOfObject', literal(type(_,O1d))),atom_to_term(O1d,ID,_),
    rdf(InnerObj,'http://knowrob.org/kb/knowrob.owl#widthOfObject', literal(type(_,O1w))),atom_to_term(O1w,IW,_),
    rdf(InnerObj,'http://knowrob.org/kb/knowrob.owl#heightOfObject', literal(type(_,O1h))),atom_to_term(O1h,IH,_),
    % read the dimensions of the Object2
    rdf(OuterObj,'http://knowrob.org/kb/knowrob.owl#depthOfObject', literal(type(_,O2d))),atom_to_term(O2d,OD,_),
    rdf(OuterObj,'http://knowrob.org/kb/knowrob.owl#widthOfObject', literal(type(_,O2w))),atom_to_term(O2w,OW,_),
    rdf(OuterObj,'http://knowrob.org/kb/knowrob.owl#heightOfObject', literal(type(_,O2h))),atom_to_term(O2h,OH,_),
    % InnerObj is contained by OuterObj if (center_i+0.5*dim_i)<=(center_o+0.5*dim_o)
    % for all dimensions (x, y, z)
    print(ok),
    >=( (IX - 0.5*ID), (OX - 0.5*OD)-0.05), =<( (IX + 0.5*ID),  (OX + 0.5*OD)+0.05 ),
    >=( (IY - 0.5*IW), (OY - 0.5*OW)-0.05 ), =<( (IY + 0.5*IW), (OY + 0.5*OW)+0.05 ),
    >=( (IZ - 0.5*IH), (OZ - 0.5*OH)-0.05 ), =<( (IZ + 0.5*IH), (OZ + 0.5*OH)+0.05 ),
    InnerObj \= OuterObj.



% MT: tried to use matrix transformation library to perform easier computation of 'inside'
% using bounding box. Problem; does not work as long as not both objects are bound
%
% holds(in_ContGeneric(InnerObj, OuterObj), T) :-
%
%
% TODO: take time into account
%
%     nonvar(InnerObj), nonvar(OuterObj),
%     transform_relative_to(InnerObj, OuterObj, [_,_,_,IrelOX,_,_,_,IrelOY,_,_,_,IrelOZ,_,_,_,_]),
%
%     % read the dimensions of the outer entity
%     rdf_triple(knowrob:widthOfObject, OuterObj, LOW),strip_literal_type(LOW,Ow),atom_to_term(Ow,OW,_),
%     rdf_triple(knowrob:heightOfObject,OuterObj, LOH),strip_literal_type(LOH,Oh),atom_to_term(Oh,OH,_),
%     rdf_triple(knowrob:depthOfObject, OuterObj, LOD),strip_literal_type(LOD,Od),atom_to_term(Od,OD,_),
%
%
%     % is InnerInOuterCoordList inside bounding box of outer object?
%
%     >=( OD, IrelOX),
%     >=( OW, IrelOY),
%     >=( OH, IrelOZ),
%
%     InnerObj \= OuterObj.



% % % % % % % % % % % % % % % % % % % %
% matrix and vector computations (relating the homography-based
% position representation with the old center-point-based one)
%

%% comp_center(+Obj, ?Center) is semidet.
%
% Compute the center point of an object from its homography matrix
%
% @param Obj    The object identifier
% @param Center The center point identifier as a String 'translation_<rotation matrix identifier>'
    comp_center(Obj, Center) :-
      rdf_triple(knowrob:orientation, Obj, Matrix),
      rdf_split_url(G, L, Matrix),
      atom_concat('translation_', L, P),
      rdf_split_url(G, P, Center).



      
%% objectAtPoint2D(+Point2D, ?Obj) is nondet.
%
% Compute which objects are positioned at the (x,y) coordinate of Point2D
%
% @param Point2D  Instance of a knowrob:Point2D for which the xCoord and yCoord can be computed
% @param Obj      Objects whose bounding boxes overlap this point in x,y direction
% 
objectAtPoint2D(Point2D, Obj) :-
    % get coordinates of point of interest
    rdf_triple(knowrob:xCoord, Point2D, PCxx), strip_literal_type(PCxx, PCx), atom_to_term(PCx,PX,_),
    rdf_triple(knowrob:yCoord, Point2D, PCyy), strip_literal_type(PCyy, PCy), atom_to_term(PCy,PY,_),
    objectAtPoint2D(PX,PY,Obj).

%
%

%% objectAtPoint2D(+PX, +PY, ?Obj) is nondet.
%
% Compute which objects are positioned at the given (x,y) coordinate 
%
% @param PX   X coordinate to be considered    
% @param PY   Y  coordinate to be considered
% @param Obj  Objects whose bounding boxes overlap this point in x,y direction
% @bug        THIS IS BROKEN FOR ALL NON-STANDARD ROTATIONS if the upper left matrix is partly zero
%
objectAtPoint2D(PX,PY,Obj) :-

    % get information of potential objects at positon point2d (x/y)

    rdf_triple(knowrob:depthOfObject, Obj, Oww), strip_literal_type(Oww, Ow),atom_to_term(Ow,OW,_),
    rdf_triple(knowrob:widthOfObject, Obj, Odd), strip_literal_type(Odd, Od),atom_to_term(Od,OD,_),

    rdf_triple(knowrob:orientation, Obj, Mat),
    rdf_triple(knowrob:m03, Mat, Tmm03), strip_literal_type(Tmm03, TM03),atom_to_term(TM03,OX,_),
    rdf_triple(knowrob:m13, Mat, Tmm13), strip_literal_type(Tmm13, TM13),atom_to_term(TM13,OY,_),
    rdf_triple(knowrob:m00, Mat, Tmm00), strip_literal_type(Tmm00, TM00),atom_to_term(TM00,M00,_),
    rdf_triple(knowrob:m01, Mat, Tmm01), strip_literal_type(Tmm01, TM01),atom_to_term(TM01,M01,_),
    rdf_triple(knowrob:m10, Mat, Tmm10), strip_literal_type(Tmm10, TM10),atom_to_term(TM10,M10,_),
    rdf_triple(knowrob:m11, Mat, Tmm11), strip_literal_type(Tmm11, TM11),atom_to_term(TM11,M11,_),

    % object must have an extension
    <(0,OW), <(0,OD),

    % calc corner points of rectangle (consider rectangular objects only!)
    P0X is (OX - 0.5*OW),
    P0Y is (OY + 0.5*OD),
    P1X is (OX + 0.5*OW),
    P1Y is (OY + 0.5*OD),
    P2X is (OX - 0.5*OW),
    P2Y is (OY - 0.5*OD),
    % rotate points
    RP0X is (OX + (P0X - OX) * M00 + (P0Y - OY) * M01),
    RP0Y is (OY + (P0X - OX) * M10 + (P0Y - OY) * M11),
    RP1X is (OX + (P1X - OX) * M00 + (P1Y - OY) * M01),
    RP1Y is (OY + (P1X - OX) * M10 + (P1Y - OY) * M11),
    RP2X is (OX + (P2X - OX) * M00 + (P2Y - OY) * M01),
    RP2Y is (OY + (P2X - OX) * M10 + (P2Y - OY) * M11),

    % debug: print rotated points
    %write('P0 X: '), write(P0X), write(' -> '), writeln(RP0X),
    %write('P0 Y: '), write(P0Y), write(' -> '), writeln(RP0Y),
    %write('P1 X: '), write(P1X), write(' -> '), writeln(RP1X),
    %write('P1 Y: '), write(P1Y), write(' -> '), writeln(RP1Y),
    %write('P2 X: '), write(P2X), write(' -> '), writeln(RP2X),
    %write('P2 Y: '), write(P2Y), write(' -> '), writeln(RP2Y),

    V1X is (RP1X - RP0X),
    V1Y is (RP1Y - RP0Y),

    V2X is (RP2X - RP0X),
    V2Y is (RP2Y - RP0Y),

    VPX is (PX - RP0X),
    VPY is (PY - RP0Y),

    DOT1 is (VPX * V1X + VPY * V1Y),
    DOT2 is (VPX * V2X + VPY * V2Y),
    DOTV1 is (V1X * V1X + V1Y * V1Y),
    DOTV2 is (V2X * V2X + V2Y * V2Y),

    =<(0,DOT1), =<(DOT1, DOTV1),
    =<(0,DOT2), =<(DOT2, DOTV2).


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
    

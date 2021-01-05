:- module(arbi_SpeechPerception,
    [   
        currentSpeechPerception/1,
        latest_detection_of_SpeechPerception/2,
        detection_starttime/2
    ]).






currentSpeechPerception(PerceptionList) :-
      rdfs_individual_of(Object,  knowrob:'Person'),
      latest_detection_of_SpeechPerception(Object, SpeechPerception),
      append([], SpeechPerception, PerceptionList).



detection_starttime(Detection, StartTime) :-

  % start time is asserted
  rdf_triple(knowrob:startTime, Detection, StartTtG),
  rdf_split_url(_, StartTt, StartTtG),
  atom_concat('timepoint_', StartTAtom, StartTt),
  term_to_atom(StartTime, StartTAtom),! ;

  rdf_split_url(_, StartTt, Detection),
  atom_concat('timepoint_', StartTAtom, StartTt),
  term_to_atom(StartTime, StartTAtom).


compare_object_detections(Delta, P1, P2) :-

    nth0(2, P1, St1),
    nth0(2, P2, St2),
    compare(Delta, St2, St1).

latest_detection_of_SpeechPerception(Object, LatestDetection) :-


   % old version without linked list of detections
    findall([D_i,Object,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              rdfs_individual_of(D_i,  arbi:'SpeechPerception'),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),
    
    % compute the homography for the newest perception
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection).

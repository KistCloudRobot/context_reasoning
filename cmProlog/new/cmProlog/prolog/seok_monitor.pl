:- dynamic monitor/1.

start_monitor :-
	rdf_monitor(monitor,
		    [ -assert(load)
		    ]), nb_getval(cm, CM).
stop_monitor :-
	rdf_monitor(monitor,
		    [ -all
		    ]).
		    
monitor(assert(S, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#LocationPerception', DB)) :-
	jpl_get('java.lang.System', out, O), jpl_call(O, println, ['static monitor occured !!'],CM),
	rdf(S, 'http://knowrob.org/kb/knowrob.owl#objectActedOn', RobIns),
	robot_locatedIn(RobIns,'http://knowrob.org/kb/demo.owl#robby-Room1',T),
	rdf_assert('http://knowrob.org/kb/demo.owl#robot999', 'http://ailab.kyonggi.ac.kr#isArrivedDestination', literal(type(xsd:boolean, true))).

	

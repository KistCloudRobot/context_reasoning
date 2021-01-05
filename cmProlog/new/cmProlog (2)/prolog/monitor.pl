:- dynamic monitor/1.

start_monitor :-
      rdf_monitor(monitor,
                  [+assert
                  ]).
stop_monitor :-
      rdf_monitor(monitor,
                  [+all
                  ]).


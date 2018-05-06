-module(rvo2_worker).
-author('lbaihbc@gmail.com').

-export([]).

init(Start, End, DoneEvent) ->
	#rvo2_worker{start = Start, end = End, doneEvent = DoneEvent}.

config(Start, End, Rvo2Worker) ->
	Rvo2Worker#rvo2_worker{start = Start, end = End}.

step(Rvo2Worker = #rvo2_worker{start = Start, end = End, doneEvent = DoneEvent}) ->
	

	lists:seq(Start, End).
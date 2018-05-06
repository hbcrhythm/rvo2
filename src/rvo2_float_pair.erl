-module(rvo2_float_pair).
-author('labihbc@gmail.com').

-include("rvo2.hrl").

-export([init/2, lt/2, le/2, gt/2, ge/2]).

init(A, B) ->
	#rvo2_float_pair{a = A, b = B}.

lt(#rvo2_float_pair{a = A1, b = B1}, #rvo2_float_pair{a = A2, b = B2}) ->
	A1 < A2 orelse (not A2 < A1) andalso B1 < B2.

le(A = #rvo2_float_pair{a = A1, b = B1}, B = #rvo2_float_pair{a = A2, b = B2}) ->
	A1 == A2 andalso B1 == B2 orelse lt(A, B).

gt(A, B) ->
	not le(A, B).

ge(A, B) ->
	not lt(A, B).

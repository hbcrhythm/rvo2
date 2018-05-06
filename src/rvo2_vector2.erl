-module(rvo2_vector2).
-author('labihbc@gmail.com').

-include("rvo2.hrl").

-export([init/2, dot/2, multiply/2, divide/2, add/2, subtract/2, negative/1]).


init(X, Y) ->
	#rvo2_vector{x = X, y = Y}.

dot(#rvo2_vector{x = AX, y = AY}, #rvo2_vector{x = BX, y = BY}) ->
	#rvo2_vector{x = AX * BX, y = AY * BY}.

multiply(#rvo2_vector{x = AX, y = AY}, #rvo2_vector{x = BX, y = BY}) ->
	AX * BX + AY * BY.
multiply(Scalar, Vector2) when is_integer(Scalar) ->
	multiply(Vector2, Scalar);
multiply(#rvo2_vector{x = X, y = Y}, Scalar) ->
	#rvo2_vector{x = X * Scalar, y = Y * Scalar}.

divide(Scalar, Vector2) when is_integer(Scalar) ->
	divide(Vector2, Scalar);
divide(#rvo2_vector{x = X, y = Y}, Scalar) ->
	#rvo2_vector{x = X / Scalar, y = Y / Scalar}.

add(#rvo2_vector{x = AX, y = AY}, #rvo2_vector{x = BX, y = BY}) ->
	#rvo2_vector{x = AX + BX, y = AY + BY}.

subtract(#rvo2_vector{x = AX, y = AY}, #rvo2_vector{x = BX, y = BY}) ->
	#rvo2_vector{x = AX - BX, y = AY - BY}.

negative(#rvo2_vector{x = X, y = Y}) ->
	#rvo2_vector{x = -X, y = -Y}.
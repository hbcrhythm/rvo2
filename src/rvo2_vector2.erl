-module(rvo2_vector2).
-author('labihbc@gmail.com').

-include("rvo2.hrl").

-export([init/2, mult/2, divide/2, add/2, sub/2, negative/1]).


init(X, Y) ->
	#rvo2_vector{x = X, y = Y}.

mult(#rvo2_vector{x = AX, y = AY}, #rvo2_vector{x = BX, y = BY}) ->
	AX * BX + AY * BY;
mult(Scalar, Vector2 = #rvo2_vector{}) ->
	mult(Vector2, Scalar);
mult(#rvo2_vector{x = X, y = Y}, Scalar) ->
	#rvo2_vector{x = X * Scalar, y = Y * Scalar}.

divide(Scalar, Vector2) when is_integer(Scalar) ->
	divide(Vector2, Scalar);
divide(#rvo2_vector{x = X, y = Y}, Scalar) ->
	#rvo2_vector{x = X / Scalar, y = Y / Scalar}.

add(#rvo2_vector{x = AX, y = AY}, #rvo2_vector{x = BX, y = BY}) ->
	#rvo2_vector{x = AX + BX, y = AY + BY}.

sub(#rvo2_vector{x = AX, y = AY}, #rvo2_vector{x = BX, y = BY}) ->
	#rvo2_vector{x = AX - BX, y = AY - BY}.

negative(#rvo2_vector{x = X, y = Y}) ->
	#rvo2_vector{x = -X, y = -Y}.
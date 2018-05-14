-module(rvo2_match).
-author('labihbc@gmail.com').

-include("rvo2.hrl").

-export([abs/1, absSq/1, normalize/1,det/2, distSqPointLineSegment/3, fabs/1, leftOf/3, sqr/1, sqrt/1]).

abs(Vector2) ->
	sqrt(absSq(Vector2)).

absSq(Vector2) ->
	rvo2_vector2:dot(Vector2, Vector2).

normalize(Vector2) ->
	rvo2_vector2:divide(Vector2, rvo2_match:abs(Vector2)).

%% @doc Computes the determinant of a two-dimensional square matrix	with rows consisting of the specified two-dimensional vectors.
det(#rvo2_vector{x = AX, y = AY}, #rvo2_vector{x = BX, y = BY}) ->
	AX * BY - AY * BX.

distSqPointLineSegment(Vector1, Vector2, Vector3) ->
	R = rvo2_vector2:divide(rvo2_vector2:multiply(rvo2_vector2:subtract(Vector3, Vector1) , rvo2_vector2:subtract(Vector2, Vector1)), absSq(rvo2_vector2:subtract(Vector2, Vector1)) ),

	if
		R < 0.0 ->
			absSq(rvo2_vector2:subtract(Vector3, Vector1));
		R > 1.0 ->
			absSq(rvo2_vector2:subtract(Vector3, Vector2));
		true ->
			absSq(rvo2_vector2:subtract(Vector3, rvo2_vector2:add(Vector1, rvo2_vector2:multiply(rvo2_vector2:subtract(Vector2, Vector1), R))))
	end.

fabs(Scalar) ->
	math:abs(Scalar).

%% @doc Computes the signed distance from a line connecting the specified points to a specified point. Return positive when the point c lies to the left of the line ab.
%% @type A :: rvo2_vector  The first point on the line
%% @type B :: rvo2_vector  The second point on the line
%% @type C :: rvo2_vector  The point to which the signed distance is to be calculated
leftOf(A, B, C) ->
	det(rvo2_vector2:subtract(A, C), rvo2_vector2:subtract(B, A)).

sqr(Scalar) ->
	Scalar * Scalar.

sqrt(Scalar) ->
	math:sqrt(Scalar).
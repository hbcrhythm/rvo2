-module(rvo2_agent).
-author('labihbc@gmail.com').

-include("rvo2.hrl").

-export([computeNeighbors/2, insertObstacleNeighbor/3]).

computeNeighbors(RvoSimulator = #rvo2_simulator{kdTree = KdTree}, Rvo2Agent = #rvo2_agent{timeHorizonObst = TimeHorizonObst, maxSpeed = MaxSpeed, radius = Radius}) ->
	RangeSq = rvo2_match:sqr(TimeHorizonObst * MaxSpeed + Radius),

insertObstacleNeighbor(Rvo2Obstacle  = #rvo2_obstacle{next = NextObstacle}, RangeSq, Rvo2Agent = #rvo2_agent{position = Position, obstacleNeighbors = ObstacleNeighbors}) ->
	DistSq = rvo2_match:distSqPointLineSegment(Rvo2Obstacle#rvo2_obstacle.point, NextObstacle#rvo2_obstacle.point, Position),

	case DistSq < RangeSq of
		true ->
			ObstacleNeighbors2 = [{DistSq, Rvo2Obstacle} | ObstacleNeighbors],

			F = fun({A, _}, {B, _}) ->
				A > B
			end,	
			ObstacleNeighbors3 = lists:sort(F, ObstacleNeighbors2),
			Rvo2Agent#rvo2_agent{obstacleNeighbors = ObstacleNeighbors3};
		false ->
			Rvo2Agent
	end.

computeNewVelocity(TimeStep, Rvo2Agent = #rvo2_agent{agentNeighbors = AgentNeighbors, orcaLines = _OrcaLines, timeHorizon = TimeHorizon, timeHorizonObst = TimeHorizonObst, obstacleNeighbors = ObstacleNeighbors, position = Position, radius = Radius, velocity = Velocity, maxSpeed = MaxSpeed, prefVelocity = PrefVelocity, newVelocity = NewVelocity}) ->
	InvTimeHorizonObst = 1.0 / TimeHorizonObst,
	
	OrcaLines = [],

	F = fun({DistSq, Obstacle1 = #rvo2_obstacle{next = Obstacle2}}, Acc) ->
	
		RelativePosition1 = rvo2_vector2:subtract(Obstacle1#rvo2_obstacle.point, Position),
		RelativePosition2 = rvo2_vector2:subtract(Obstacle2#rvo2_obstacle.point, Position),

		AlreadyCovered = alreadyCovered(Acc, InvTimeHorizonObst, RelativePosition1, RelativePosition2, Rvo2Agent)

		case AlreadyCovered of
			true ->
				Acc;
			false ->
				%% Not yet covered, Check for Collisions.

				DistSq1 = rvo2_match:absSq(RelativePosition1),
				DistSq2 = rvo2_match:absSq(RelativePosition2),
				RadiusSq= rvo2_match:srq(Radius),

				ObstacleVector = Obstacle2#rvo2_obstacle.point - Obstacle1#rvo2_obstacle.point,

				S = (-RelativePosition1 * ObstacleVector) / rvo2_match:absSq(ObstacleVector),
				DistSqLine = rvo2_match:absSq(rvo2_vector2:subtract(-RelativePosition1, rvo2_vector2:multiply(S, ObstacleVector))),

				if
					S < 0.0 andalso DistSqLine =< RadiusSq  ->
						case Obstacle1#rvo2_obstacle.convex of
							true ->
								Rvo2Line = #rvo2_line{point = rvo2_vector2:init(0.0, 0.0), direction = rvo2_match:normalize(rvo2_vector2:init(-RelativePosition1#rvo2_vector.y, RelativePosition1#rvo2_vector.x))},
								[Rvo2Line | Acc];
							false ->
								Acc
						end;
					S > 1.0 andalso DistSq2 =< RadiusSq ->
						case Obstacle1#rvo2_obstacle.convex andalso rvo2_match:det(RelativePosition2, Obstacle2#rvo2_obstacle.direction) >= 0.0 of
							true ->
								Rvo2Line = #rvo2_line{point = rvo2_vector2:init(0.0, 0.0), direction = rvo2_match:normalize(rvo2_vector2:init(-RelativePosition2#rvo2_vector.y, RelativePosition2#rvo2_vector.x))},
								[Rvo2Line | Acc];
							false ->
								Acc
						end;
					S >= 0.0 andalso S < 1.0 andalso DistSqLine =< RadiusSq ->
						Rvo2Line = #rvo2_line{point = rvo2_vector2:init(0.0, 0.0), direction = -Obstacle1#rvo2_obstacle.direction},
						[Rvo2Line | Acc];
					true ->
						{LeftLegDirection3, RightLegDirection3, NewObstacle1, NewObstacle2} = if
							S < 0.0 andalso DistSqLine =< RadiusSq ->
								case not Obstacle1#rvo2_obstacle.convex of
									true ->
										{undefined, undefined, Obstacle1, Obstacle2};
									false ->
										Leg1 = rvo2_match:sqrt(DistSq1 - RadiusSq),
										LeftLegDirection  = rvo2_vector2:init(RelativePosition1#rvo2_vector.x * Leg1 -  RelativePosition1#rvo2_vector.y * Radius, RelativePosition1#rvo2_vector.x * Radius + RelativePosition1#rvo2_vector.y * Leg1) / DistSq1,
										RightLegDirection = rvo2_vector2:init(RelativePosition1#rvo2_vector.x * Leg1 +  RelativePosition1#rvo2_vector.y * Radius, -RelativePosition1#rvo2_vector.x * Radius + RelativePosition1#rvo2_vector.y * Leg1) / DistSq1,
										{LeftLegDirection, RightLegDirection, Obstacle1, Obstacle1}
								end;
							S > 1.0 andalso DistSqLine =< RadiusSq ->
								case not Obstacle2#rvo2_obstacle.convex of
									true ->
										{undefined, undefined, Obstacle1, Obstacle2};
									false ->
										Leg2 = rvo2_match:sqrt(DistSq2 - RadiusSq),
										LeftLegDirection  = rvo2_vector2:init(RelativePosition2#rvo2_vector.x * Leg2 -  RelativePosition2#rvo2_vector.y * Radius, RelativePosition2#rvo2_vector.x * Radius + RelativePosition2#rvo2_vector.y * Leg2) / DistSq2,
										RightLegDirection = rvo2_vector2:init(RelativePosition2#rvo2_vector.x * Leg2 +  RelativePosition2#rvo2_vector.y * Radius, -RelativePosition2#rvo2_vector.x * Radius + RelativePosition2#rvo2_vector.y * Leg2) / DistSq2
										{LeftLegDirection, RightLegDirection, Obstacle2, Obstacle2}
								end;
							true ->
								LeftLegDirection2 = case Obstacle1#rvo2_obstacle.convex of
									true ->
										Leg1 = rvo2_match:sqrt(DistSq1 - RadiusSq),
										LeftLegDirection  = rvo2_vector2:init(RelativePosition1#rvo2_vector.x * Leg1 -  RelativePosition1#rvo2_vector.y * Radius, RelativePosition1#rvo2_vector.x * Radius + RelativePosition1#rvo2_vector.y * Leg1) / DistSq1;
									false ->
										LeftLegDirection = -Obstacle1#rvo2_obstacle.direction
								end,
								RightLegDirection2 = case Obstacle2#rvo2_obstacle.convex of
									true ->
										Leg2 = rvo2_match:sqrt(DistSq2 - RadiusSq),
										RightLegDirection = rvo2_vector2:init(RelativePosition2#rvo2_vector.x * Leg2 +  RelativePosition2#rvo2_vector.y * Radius, -RelativePosition2#rvo2_vector.x * Radius + RelativePosition2#rvo2_vector.y * Leg2) / DistSq2
									false ->
										RightLegDirection = Obstacle1#rvo2_obstacle.direction
								end,
								{LeftLegDirection2, RightLegDirection2, Obstacle1, Obstacle2}

						end,

						case {LeftLegDirection3, RightLegDirection3} of
							{undefined, _} ->
								Acc;
							{_, undefined} ->
								Acc
							_ ->
				                %% Legs can never point into neighboring edge when convex
				                %% vertex, take cutoff-line of neighboring edge instead. If
				                %% velocity projected on "foreign" leg, no constraint is added.
				                LeftNeighbor = NewObstacle1#rvo2_obstacle.previous,
				                % IsLeftLegForeign = false,
				                % IsRightLegForeign = false,
				                {IsLeftLegForeign, LeftLegDirection4} = case NewObstacle1#rvo2_obstacle.convex andalso rvo2_match:det(LeftLegDirection3, rvo2_vector2:negative(LeftNeighbor#rvo2_obstacle.direction)) >= 0.0 of
				                	true ->
				                		% IsRightLegForeign2 = true,
				                		{true, rvo2_vector2:negative(LeftNeighbor#rvo2_obstacle.direction)};
				                	false ->
				                		{false, LeftLegDirection3}
				                end,
				                {IsRightLegForeign, RightLegDirection4} = case NewObstacle2#rvo2_obstacle.convex andalso rvo2_match:det(RightLegDirection3, NewObstacle2#rvo2_obstacle.direction) =< 0.0 of
				                	true ->
				                		% IsRightLegForeign2 = true,
				                		{true, NewObstacle2#rvo2_obstacle.direction};
				                	false ->
				                		{false, RightLegDirection3}
				                end,

				                LeftCutOff 	= InvTimeHorizonObst * (rvo2_vector2:subtract( NewObstacle1#rvo2_obstacle.point , Position)),
				                RightCutOff	= InvTimeHorizonObst * (rvo2_vector2:subtract( NewObstacle2#rvo2_obstacle.point , Position)),

				                CutOffVector = rvo2_vector2:subtract(RightCutOff, LeftCutOff),

				                T 		= ?RVO2_IF( NewObstacle1 == NewObstacle2 , 0.5 , ((Velocity - LeftCutOff) * CutOffVector) / rvo2_match:absSq(CutOffVector)),
								TLeft 	= rvo2_vector2:multiply(rvo2_vector2:subtract(Velocity , LeftCutOff) , LeftLegDirection4),
								TRight 	= rvo2_vector2:multiply(rvo2_vector2:subtract(Velocity, RightCutOff) , RightLegDirection4),

								if
									(T < 0.0 andalso TLeft < 0.0) orelse (NewObstacle1 == NewObstacle2 andalso TLeft < 0.0 andalso TRight < 0.0) ->
										UnitW = rvo2_match:normalize( rvo2_vector2:subtract(Velocity, LeftCutOff) ),
										Rvo2Line = #rvo2_line{direction = rvo2_vector2:init(UnitW#rvo2_vector2.y, -UnitW#rvo2_vector2.x), point = rvo2_vector2:add(LeftCutOff , rvo2_vector2:multiply(rvo2_vector2:multiply(Radius, InvTimeHorizonObst ), UnitW))},
										[Rvo2Line | Acc];
									(T > 1.0 andalso TRight < 0.0) ->
										UnitW = rvo2_match:normalize( rvo2_vector2:subtract(Velocity, RightCutOff) ),
										Rvo2Line = #rvo2_line{direction = rvo2_vector2:init(UnitW#rvo2_vector2.y, -UnitW#rvo2_vector2.x), point = rvo2_vector2:add(RightCutOff , rvo2_vector2:multiply(rvo2_vector2:multiply(Radius, InvTimeHorizonObst ), UnitW))},
										[Rvo2Line | Acc];
									true ->
										DistSqCutoff = ?RVO2_IF( (T < 0.0 orelse T > 1.0 orelse NewObstacle1 == NewObstacle2), ?MAX_VALUE , rvo2_vector2:absSq(rvo2_vector2:subtract(Velocity, (rvo2_vector2:add(LeftCutOff , rvo2_vector2:multiply(T , CutOffVector))))) ),
										DistSqLeft = ?RVO2_IF( TLeft < 0.0, ?MAX_VALUE, rvo2_vector2:absSq(rvo2_vector2:subtract(Velocity, (rvo2_vector2:add(LeftCutOff , rvo2_vector2:multiply(TLeft , LeftLegDirection4))))) ),
										DistSqRight = ?RVO2_IF( TRight < 0.0, ?MAX_VALUE, rvo2_vector2:absSq(rvo2_vector2:subtract(Velocity, (rvo2_vector2:add(RightCutOff , rvo2_vector2:multiply(TRight , RightLegDirection4))))) ),

										case DistSqCutoff =< DistSqLeft andalso DistSqCutoff =< DistSqRight of
											true ->
												Direction = rvo2_vector2:negative(NewObstacle1#rvo2_obstacle.direction),
												Rvo2Line = #rvo2_line{direction = Direction, point = rvo2_vector2:add(LeftCutOff , rvo2_vector2:multiply(rvo2_vector2:multiply(Radius, InvTimeHorizonObst ), rvo2_vector2:init(-Direction#rvo2_vector2.y, Direction#rvo2_vector2.x)))},
												[Rvo2Line | Acc];
											false ->
												case DistSqCutoff =< DistSqRight of
													true ->
														case IsLeftLegForeign of
															true ->
																Acc;
															false ->
																Direction = LeftLegDirection3,
																Rvo2Line = #rvo2_line{direction = Direction, point = rvo2_vector2:add(LeftCutOff , rvo2_vector2:multiply(rvo2_vector2:multiply(Radius, InvTimeHorizonObst ), rvo2_vector2:init(-Direction#rvo2_vector2.y, Direction#rvo2_vector2.x)))},
																[Rvo2Line | Acc]
														end;
													false ->
														case IsRightLegForeign of
															true ->
																Acc;
															false ->
																Direction = -RightLegDirection3,
																Rvo2Line = #rvo2_line{direction = Direction, point = rvo2_vector2:add(RightCutOff , rvo2_vector2:multiply(rvo2_vector2:multiply(Radius, InvTimeHorizonObst ), rvo2_vector2:init(-Direction#rvo2_vector2.y, Direction#rvo2_vector2.x)))},
																[Rvo2Line | Acc]
														end
												end
										end
								end
						end
				end				
		end
	end,
	OrcaLines2 = lists:foldl(F, OrcaLines, ObstacleNeighbors), 
	
	Len = length(OrcaLines2),

	InvTimeHorizon2 = 1.0 / TimeHorizon,

	FF = fun({_, Other}, Acc) ->
		
		RelativePosition = rvo2_vector2:subtract(Other#rvo2_agent.position, Position),
		RelativeVelocity = rvo2_vector2:subtract(Velocity, Other#rvo2_agent.velocity),
		DistSq = rvo2_match:absSq(RelativePosition),
		CombinedRadius = Radius + Other#rvo2_agent.radius,
		CombinedRadiusSq = rvo2_match:sqr(CombinedRadius),

		Line = #rvo2_line{},

		% U = #rvo2_vector2{},

		{Direction2, U} = case DistSq > CombinedRadiusSq of
			true ->
				W = rvo2_vector2:subtract(RelativeVelocity - rvo2_vector2:multiply(InvTimeHorizon2 , RelativePosition)),

				WLengthSq = rvo2_match:absSq(W),
				DotProduct1 =rvo2_vector2:multiply(W, RelativePosition),

				case DotProduct1 < 0.0 andalso rvo2_match:sqr(DotProduct1) > CombinedRadiusSq * WLengthSq of
					true ->
						WLength = rvo2_match:sqrt(WLengthSq),
						UnitW 	= rvo2_vector2:divide(W, WLength),
						{rvo2_vector2:init(UnitW#rvo2_vector2.y, UnitW#rvo2_vector2.x), rvo2_vector2:multiply(CombinedRadius * InvTimeHorizon2 - WLength, UnitW)};
					false ->
						Leg = rvo2_match:sqrt(DistSq - CombinedRadiusSq),
						Direction = case rvo2_match:det(RelativePosition, W) > 0.0 of
							true ->
								rvo2_vector2:init(RelativePosition#rvo2_vector2.x * Leg - RelativePosition#rvo2_vector2.y * CombinedRadius, RelativePosition#rvo2_vector2.x * CombinedRadius + RelativePosition#rvo2_vector2.y * Leg);	
							false ->
								rvo2_vector2:negative(rvo2_vector2:init(RelativePosition#rvo2_vector2.x * Leg - RelativePosition#rvo2_vector2.y * CombinedRadius, -RelativePosition#rvo2_vector2.x * CombinedRadius + RelativePosition#rvo2_vector2.y * Leg));	
						end,
						{Direction, rvo2_vector2:subtract( rvo2_vector2:multiply(rvo2_vector2:multiply(RelativeVelocity, Direction) * Direction), RelativeVelocity)}
				end;
			false ->
				InvTimeStep = 1.0 / TimeStep,
				W = rvo2_vector2:subtract(RelativeVelocity - rvo2_vector2:multiply(InvTimeStep , RelativePosition)),
				WLength = rvo2_match:abs(W),
				UnitW 	= rvo2_vector2:divide(W, WLength),
				Direction = rvo2_vector2:init(UnitW#rvo2_vector2.y , - UnitW#rvo2_vector2.x),

				{Direction, rvo2_vector2:multiply( CombinedRadius * InvTimeStep - WLength, UnitW)}
		end,
		Line2 = Line#rvo2_vector2{point = rvo2_vector2:add(Velocity, rvo2_vector2:multiply(0.5, U)) },
		[Line2 | Acc]
	end,
	OrcaLines3 = lists:foldl(FF, OrcaLines2, AgentNeighbors),

	{LineFail, Result} = linearProgram2(OrcaLines3, MaxSpeed, PrefVelocity, false, NewVelocity),
		
	case LineFail < length(OrcaLines3) of
		true ->
			linearProgram3(OrcaLines3, Len, LineFail, MaxSpeed)
		false ->
			Result
	end.




alreadyCovered([ OrcaLine | T], InvTimeHorizonObst, RelativePosition1, RelativePosition2, Rvo2Agent = #rvo2_agent{radius = Radius}) ->
	case
		rvo2_match:det( rvo2_vector2:subtract( rvo2_vector2:multiply(InvTimeHorizonObst, RelativePosition1), OrcaLine#rvo2_line.point), OrcaLine#rvo2_line.direction) - InvTimeHorizonObst * Radius >= -?RVO_EPSILON 
	andalso 
		rvo2_match:det( rvo2_vector2:subtract( rvo2_vector2:multiply(InvTimeHorizonObst, RelativePosition2), OrcaLine#rvo2_line.point), OrcaLine#rvo2_line.direction) - InvTimeHorizonObst * Radius >= -?RVO_EPSILON
	of
		true ->
			true;
		false ->
			isAlreadyCovered(T, InvTimeHorizonObst, RelativePosition1, RelativePosition2, Rvo2Agent)
	end.

linearProgram1(Lines, LineNo, Radius, OptVelocity, DirectionOpt) ->
	#rvo2_line{point = Point, direction = Direction} = lists:nth(LineNo, Lines),
	DotProduct = rvo2_vector2:multiply( Point, Direction),
	Discriminant = rvo2_match:sqr(DotProduct) + rvo2_match:sqr(Radius) -  rvo2_match:absSq(Point),
	case Discriminant < 0.0 of
		true ->
			false;
		false ->
			SqrtDiscriminant = rvo2_match:sqrt(Discriminant),
			TLeft = -DotProduct - SqrtDiscriminant,
			TRight = -DotProduct + SqrtDiscriminant,
			{Tag, TLeft2, TRight2} = do_linearProgram1(lists:seq(1, LineNo), Lines, TLeft, TRight),
			case Tag of
				false ->
					{Tag, Result};
				true ->
					case Discriminant of
						true ->
							case rvo2_vector2:multiply(OptVelocity, Direction) > 0.0  of
								true ->
									{true, rvo2_vector2:add(Point, rvo2_vector2:multiply(TRight2, Direction))},	
								false ->
									{true, rvo2_vector2:add(Point, rvo2_vector2:multiply(TLeft2, Direction))}	
							end;
						false ->
							T = rvo2_vector2:multiply(Direction, rvo2_vector2:subtract( OptVelocity, Point) ),
							if
							 	T < TLeft2 ->
							 		{true, rvo2_vector2:add(Point, rvo2_vector2:multiply(TLeft2, Direction))};
								T > TRight2 ->
									{true, rvo2_vector2:add(Point, rvo2_vector2:multiply(TRight2, Direction))};
								true ->
									{true, rvo2_vector2:add(Point, rvo2_vector2:multiply(T, Direction))};
							end
					end
			end
	end.

do_linearProgram1([], _, TLeft, TRight) -> {true, TLeft, TRight};
do_linearProgram1([H | T], Lines, TLeft, TRight) ->
	#rvo2_line{direction = Direction, point = Point} = lists:nth(H, Lines),
	Denominator = rvo2_match:det(Direction, Direction),
	Mumerator = rvo2_match:det(Direction, rvo2_match:subtract(Point, Point)),

	case rvo2_match:fabs(Denominator) =< ?RVO_EPSILON of
		true ->
			case Mumerator < 0.0 of
				true ->
					{false, TLeft, TRight};	
				false ->
					do_linearProgram1(T, Lines, TLeft, TRight)
			end;
		false ->
			T = Mumerator / Denominator,

			{TLeft2, TRight2} = case Denominator >= 0.0 of
				true ->
					{TLeft, min(TRight, T)};
				false ->
					{max(TLeft, T), TRight}
			end,

			case TLeft2 > TRight2 of
				true ->
					{false, TLeft2, TRight2};
				false ->
					do_linearProgram1(T, Lines, TLeft2, TRight2)
			end
	end.



linearProgram2(Lines, Radius, OptVelocity, DirectionOpt, Result) ->
	if
	 	DirectionOpt ->
	 		rvo2_vector2:multiply(OptVelocity, Radius);
	 	rvo2_match:absSq(OptVelocity) > rvo2_match:sqr(Radius) ->
	 		rvo2_vector2:multiply( rvo2_match:normalize(OptVelocity), Radius);
		true ->
			OptVelocity
	end,
	{Count, Result2} = do_linearProgram2(lists:seq(1, length(Lines)), Lines, Radius, OptVelocity, DirectionOpt, Result)
	case Count of
		false ->
			{length(Lines), Result2};
		_ ->
			{Tag, Result2}
	end.

do_linearProgram2([], _, _, _, _, Result) -> {false, Result}.
do_linearProgram2([H | T], Lines, Radius, OptVelocity, DirectionOpt, Result) ->
	Line = lists:nth(H, Lines),
	case rvo2_match:det(Line#rvo2_line.direction, rvo2_vector2:subtract(Line#rvo2_line.point, Result)) > 0.0 of
		true ->
			{Tag, Result2} = linearProgram1(Lines, H, Radius, OptVelocity, DirectionOpt, Result),
			case not Tag of
				true ->
					{H, Result}	
				false ->
					do_linearProgram2(T, Lines, Radius, OptVelocity, DirectionOpt, Result2)
			end;
		false ->
			do_linearProgram2(T, Lines, Radius, OptVelocity, DirectionOpt, Result)
	end.
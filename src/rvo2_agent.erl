-module(rvo2_agent).
-author('labihbc@gmail.com').

-include("rvo2.hrl").

-export([computeNeighbors/2, insertObstacleNeighbor/4, insertAgentNeighbor/3, computeNewVelocity/3, update/2]).

computeNeighbors(#rvo2_simulator{kdTree = KdTree}, Agent = #rvo2_agent{timeHorizonObst = TimeHorizonObst, maxSpeed = MaxSpeed, radius = Radius}) ->

	RangeSq = rvo2_match:sqr(TimeHorizonObst * MaxSpeed + Radius),
	Agent2 = #rvo2_agent{maxNeighbors = MaxNeighbors, neighborDist = NeighborDist} = rvo2_kd_tree:computeObstacleNeighbors(Agent#rvo2_agent{obstacleNeighbors = []}, RangeSq, KdTree),
	Agent3 = Agent2#rvo2_agent{agentNeighbors = []},
	case MaxNeighbors > 0 of
		true ->
			RangeSq2 = rvo2_match:sqr(NeighborDist),

			% lager:info("id ~w Agent3#rvo2_agent.agentNeighbors ~w ~n", [Agent#rvo2_agent.id, Agent3#rvo2_agent.agentNeighbors]),

			{_RangeSq3, Agent4} = rvo2_kd_tree:computeAgentNeighbors(Agent3, RangeSq2, KdTree),

			% lager:info("id ~w _RangeSq3 ~w Agent ~w ~n",[Agent#rvo2_agent.id, _RangeSq3,  [{Id,Position} ||  #rvo2_agent{id = Id, position = Position} <- Agent4#rvo2_agent.agentNeighbors ]] ),
			% lager:info("id ~w _RangeSq3 ~w Agent ~w ~n",[Agent#rvo2_agent.id, _RangeSq3,  Agent4#rvo2_agent.agentNeighbors]),
			Agent4;
		false ->
			Agent3
	end.

update(#rvo2_simulator{timeStep = TimeStep}, Agent = #rvo2_agent{newVelocity = NewVelocity, position = Position}) ->
	
	Position2 = rvo2_vector2:add(Position,  rvo2_vector2:mult(NewVelocity, TimeStep)),

	% lager:info("rvo2agent update id = ~w NewVelocity ~w, position ~w position2 ~w TimeStep ~w ~n",[Agent#rvo2_agent.id, NewVelocity, Position, Position2, TimeStep]),

	Agent#rvo2_agent{velocity = NewVelocity, position = Position2}.

%% @doc Inserts a static obstacle neighbor into the set of neighbors of this agent
insertObstacleNeighbor(Obstacle, NextObstacle, RangeSq, Agent = #rvo2_agent{position = Position, obstacleNeighbors = ObstacleNeighbors}) ->
	DistSq = rvo2_match:distSqPointLineSegment(Obstacle#rvo2_obstacle.point, NextObstacle#rvo2_obstacle.point, Position),
	case DistSq < RangeSq of
		true ->
			ObstacleNeighbors2 = [{DistSq, Obstacle} | ObstacleNeighbors],

			ObstacleNeighbors3 = lists:keysort(1, ObstacleNeighbors2),
			
			Agent#rvo2_agent{obstacleNeighbors = ObstacleNeighbors3};
		false ->
			Agent
	end.

%% @doc Inserts an agent neighbor into the set of neighbors of this agent
insertAgentNeighbor(#rvo2_agent{id = Id}, RangeSq, Agent = #rvo2_agent{id = Id}) -> {RangeSq, Agent};
insertAgentNeighbor(InsertAgent = #rvo2_agent{position = Position2}, RangeSq, Agent = #rvo2_agent{position = Position, agentNeighbors = AgentNeighbors, maxNeighbors = MaxNeighbors}) ->
	DistSq = rvo2_match:absSq(rvo2_vector2:sub(Position, Position2)),
	case DistSq < RangeSq of
		true ->
			AgentNeighbors2 = case length(AgentNeighbors) < MaxNeighbors of
				true ->
					[{DistSq, InsertAgent} | AgentNeighbors];
				false ->
					AgentNeighbors
			end,

			AgentNeighbors3 = lists:keysort(1 , AgentNeighbors2),

			Agent2 = Agent#rvo2_agent{agentNeighbors = AgentNeighbors3},
			case length(AgentNeighbors3) == MaxNeighbors of
				true ->
					{Key, _} = lists:last(AgentNeighbors3),	
					{Key, Agent2};
				false ->
					{RangeSq, Agent2}
			end;
		false ->
			{RangeSq, Agent}
	end.


computeNewVelocity(TimeStep, Obstacles, Agent = #rvo2_agent{agentNeighbors = AgentNeighbors, orcaLines = _OrcaLines, timeHorizon = TimeHorizon, timeHorizonObst = TimeHorizonObst, obstacleNeighbors = ObstacleNeighbors, position = Position, radius = Radius, velocity = Velocity, maxSpeed = MaxSpeed, prefVelocity = PrefVelocity, newVelocity = NewVelocity}) ->
	
	% lager:info(" computeNewVelocity id = ~w AgentNeighbors ~w ~n ",[Agent#rvo2_agent.id, AgentNeighbors]), 

	InvTimeHorizonObst = 1.0 / TimeHorizonObst,
	
	OrcaLines = [],

	%% Create obstacle ORCA lines.
	F = fun({_, Obstacle1 = #rvo2_obstacle{next_id = NextId}}, Acc) ->
	
		Obstacle2 = lists:keyfind(NextId, #rvo2_obstacle.id, Obstacles),

		RelativePosition1 = rvo2_vector2:sub(Obstacle1#rvo2_obstacle.point, Position),
		RelativePosition2 = rvo2_vector2:sub(Obstacle2#rvo2_obstacle.point, Position),

		AlreadyCovered = alreadyCovered(Acc, InvTimeHorizonObst, RelativePosition1, RelativePosition2, Agent),

		case AlreadyCovered of
			true ->
				Acc;
			false ->
				%% Not yet covered, Check for Collisions.

				DistSq1 = rvo2_match:absSq(RelativePosition1),
				DistSq2 = rvo2_match:absSq(RelativePosition2),
				RadiusSq= rvo2_match:sqr(Radius),

				ObstacleVector = rvo2_vector2:sub(Obstacle2#rvo2_obstacle.point, Obstacle1#rvo2_obstacle.point),

				S = rvo2_vector2:mult(rvo2_vector2:negative(RelativePosition1), ObstacleVector) / rvo2_match:absSq(ObstacleVector),
				DistSqLine = rvo2_match:absSq(rvo2_vector2:sub(rvo2_vector2:negative(RelativePosition1), rvo2_vector2:mult(S, ObstacleVector))),

				if
					%% Collision with left vertex. Ignore if non-convex.
					S < 0.0 andalso DistSq1 =< RadiusSq  ->
						case Obstacle1#rvo2_obstacle.convex of
							true ->
								Rvo2Line = #rvo2_line{point = rvo2_vector2:init(0.0, 0.0), direction = rvo2_match:normalize(rvo2_vector2:init(-RelativePosition1#rvo2_vector.y, RelativePosition1#rvo2_vector.x))},
								[Rvo2Line | Acc];
							false ->
								Acc
						end;
					%% Collision with right vertex. Ignore if non-convex or if it will be taken care of by neighboring obstacle.
					S > 1.0 andalso DistSq2 =< RadiusSq ->
						case Obstacle1#rvo2_obstacle.convex andalso rvo2_match:det(RelativePosition2, Obstacle2#rvo2_obstacle.direction) >= 0.0 of
							true ->
								Rvo2Line = #rvo2_line{point = rvo2_vector2:init(0.0, 0.0), direction = rvo2_match:normalize(rvo2_vector2:init(-RelativePosition2#rvo2_vector.y, RelativePosition2#rvo2_vector.x))},
								
								[Rvo2Line | Acc];
							false ->
								Acc
						end;
					%% Collision with obstacle segment.
					S >= 0.0 andalso S < 1.0 andalso DistSqLine =< RadiusSq ->
						Rvo2Line = #rvo2_line{point = rvo2_vector2:init(0.0, 0.0), direction = rvo2_vector2:negative(Obstacle1#rvo2_obstacle.direction) },

						[Rvo2Line | Acc];

					%%No collision. Compute legs. When obliquely viewed, both legs can come from a single vertex. Legs extend cut-off line when non-convex vertex.
					true ->
						{LeftLegDirection3, RightLegDirection3, NewObstacle1, NewObstacle2} = if
							S < 0.0 andalso DistSqLine =< RadiusSq ->
								case not Obstacle1#rvo2_obstacle.convex of
									true ->
										{undefined, undefined, Obstacle1, Obstacle2};
									false ->
										Leg1 = rvo2_match:sqrt(DistSq1 - RadiusSq),
										LeftLegDirection  = rvo2_vector2:divide(rvo2_vector2:init(RelativePosition1#rvo2_vector.x * Leg1 -  RelativePosition1#rvo2_vector.y * Radius, RelativePosition1#rvo2_vector.x * Radius + RelativePosition1#rvo2_vector.y * Leg1), DistSq1),
										RightLegDirection = rvo2_vector2:divide(rvo2_vector2:init(RelativePosition1#rvo2_vector.x * Leg1 +  RelativePosition1#rvo2_vector.y * Radius, -RelativePosition1#rvo2_vector.x * Radius + RelativePosition1#rvo2_vector.y * Leg1), DistSq1),
										{LeftLegDirection, RightLegDirection, Obstacle1, Obstacle1}
								end;
							S > 1.0 andalso DistSqLine =< RadiusSq ->
								case not Obstacle2#rvo2_obstacle.convex of
									true ->
										{undefined, undefined, Obstacle1, Obstacle2};
									false ->
										Leg2 = rvo2_match:sqrt(DistSq2 - RadiusSq),
										LeftLegDirection  = rvo2_vector2:divide(rvo2_vector2:init(RelativePosition2#rvo2_vector.x * Leg2 -  RelativePosition2#rvo2_vector.y * Radius, RelativePosition2#rvo2_vector.x * Radius + RelativePosition2#rvo2_vector.y * Leg2), DistSq2),
										RightLegDirection = rvo2_vector2:divide(rvo2_vector2:init(RelativePosition2#rvo2_vector.x * Leg2 +  RelativePosition2#rvo2_vector.y * Radius, -RelativePosition2#rvo2_vector.x * Radius + RelativePosition2#rvo2_vector.y * Leg2), DistSq2),
										{LeftLegDirection, RightLegDirection, Obstacle2, Obstacle2}
								end;
							true ->
								LeftLegDirection2 = case Obstacle1#rvo2_obstacle.convex of
									true ->
										Leg1 = rvo2_match:sqrt(DistSq1 - RadiusSq),
										rvo2_vector2:divide(rvo2_vector2:init(RelativePosition1#rvo2_vector.x * Leg1 -  RelativePosition1#rvo2_vector.y * Radius, RelativePosition1#rvo2_vector.x * Radius + RelativePosition1#rvo2_vector.y * Leg1), DistSq1);
									false ->
										rvo2_vector2:negative(Obstacle1#rvo2_obstacle.direction)
								end,
								RightLegDirection2 = case Obstacle2#rvo2_obstacle.convex of
									true ->
										Leg2 = rvo2_match:sqrt(DistSq2 - RadiusSq),
										rvo2_vector2:divide(rvo2_vector2:init(RelativePosition2#rvo2_vector.x * Leg2 +  RelativePosition2#rvo2_vector.y * Radius, -RelativePosition2#rvo2_vector.x * Radius + RelativePosition2#rvo2_vector.y * Leg2), DistSq2);
									false ->
										Obstacle1#rvo2_obstacle.direction
								end,
								{LeftLegDirection2, RightLegDirection2, Obstacle1, Obstacle2}

						end,

						case {LeftLegDirection3, RightLegDirection3} of
							{undefined, _} ->
								Acc;
							{_, undefined} ->
								Acc;
							_ ->
				                %% Legs can never point into neighboring edge when convex
				                %% vertex, take cutoff-line of neighboring edge instead. If
				                %% velocity projected on "foreign" leg, no constraint is added.
				                LeftNeighbor = lists:keyfind(NewObstacle1#rvo2_obstacle.previous_id, #rvo2_obstacle.id, Obstacles),
				                % IsLeftLegForeign = false,
				                % IsRightLegForeign = false,
				                {IsLeftLegForeign, LeftLegDirection4} = case NewObstacle1#rvo2_obstacle.convex andalso rvo2_match:det(LeftLegDirection3, rvo2_vector2:negative(LeftNeighbor#rvo2_obstacle.direction)) >= 0.0 of
				                	true ->
				                		%% Left leg points into obstacle.
				                		{true, rvo2_vector2:negative(LeftNeighbor#rvo2_obstacle.direction)};
				                	false ->
				                		{false, LeftLegDirection3}
				                end,
				                {IsRightLegForeign, RightLegDirection4} = case NewObstacle2#rvo2_obstacle.convex andalso rvo2_match:det(RightLegDirection3, NewObstacle2#rvo2_obstacle.direction) =< 0.0 of
				                	true ->
				                		% IsRightLegForeign2 = true,
				                		%% Right leg points into obstacle.
				                		{true, NewObstacle2#rvo2_obstacle.direction};
				                	false ->
				                		{false, RightLegDirection3}
				                end,

				                LeftCutOff 	= rvo2_vector2:mult(InvTimeHorizonObst, (rvo2_vector2:sub(NewObstacle1#rvo2_obstacle.point , Position))),
				                RightCutOff	= rvo2_vector2:mult(InvTimeHorizonObst, (rvo2_vector2:sub(NewObstacle2#rvo2_obstacle.point , Position))),

				                CutOffVector = rvo2_vector2:sub(RightCutOff, LeftCutOff),

				                %% Project current velocity on velocity obstacle.
				                T 		= ?RVO2_IF( NewObstacle1 == NewObstacle2 , 0.5 , rvo2_vector2:mult(rvo2_vector2:sub(Velocity, LeftCutOff), CutOffVector) / rvo2_match:absSq(CutOffVector)),
								TLeft 	= rvo2_vector2:mult(rvo2_vector2:sub(Velocity , LeftCutOff) , LeftLegDirection4),
								TRight 	= rvo2_vector2:mult(rvo2_vector2:sub(Velocity, RightCutOff) , RightLegDirection4),

								if
									%% Project on left cut-off circle. 
									(T < 0.0 andalso TLeft < 0.0) orelse (NewObstacle1 == NewObstacle2 andalso TLeft < 0.0 andalso TRight < 0.0) ->
										UnitW = rvo2_match:normalize( rvo2_vector2:sub(Velocity, LeftCutOff) ),
										Rvo2Line = #rvo2_line{direction = rvo2_vector2:init(UnitW#rvo2_vector.y, -UnitW#rvo2_vector.x), point = rvo2_vector2:add(LeftCutOff , rvo2_vector2:mult(Radius * InvTimeHorizonObst, UnitW))},

										[Rvo2Line | Acc];
									(T > 1.0 andalso TRight < 0.0) ->
										UnitW = rvo2_match:normalize( rvo2_vector2:sub(Velocity, RightCutOff) ),
										Rvo2Line = #rvo2_line{direction = rvo2_vector2:init(UnitW#rvo2_vector.y, -UnitW#rvo2_vector.x), point = rvo2_vector2:add(RightCutOff , rvo2_vector2:mult(Radius * InvTimeHorizonObst, UnitW))},
										
										[Rvo2Line | Acc];
									true ->
										DistSqCutoff 	= ?RVO2_IF( (T < 0.0 orelse T > 1.0 orelse NewObstacle1 == NewObstacle2), ?MAX_VALUE , rvo2_match:absSq(rvo2_vector2:sub(Velocity, (rvo2_vector2:add(LeftCutOff , rvo2_vector2:mult(T , CutOffVector))))) ),
										DistSqLeft 		= ?RVO2_IF( TLeft < 0.0, ?MAX_VALUE, rvo2_match:absSq(rvo2_vector2:sub(Velocity, (rvo2_vector2:add(LeftCutOff , rvo2_vector2:mult(TLeft , LeftLegDirection4))))) ),
										DistSqRight 	= ?RVO2_IF( TRight < 0.0, ?MAX_VALUE, rvo2_match:absSq(rvo2_vector2:sub(Velocity, (rvo2_vector2:add(RightCutOff , rvo2_vector2:mult(TRight , RightLegDirection4))))) ),

										case DistSqCutoff =< DistSqLeft andalso DistSqCutoff =< DistSqRight of
											true ->
												Direction = rvo2_vector2:negative(NewObstacle1#rvo2_obstacle.direction),
												Rvo2Line = #rvo2_line{direction = Direction, point = rvo2_vector2:add(LeftCutOff , rvo2_vector2:mult(Radius * InvTimeHorizonObst, rvo2_vector2:init(-Direction#rvo2_vector.y, Direction#rvo2_vector.x)))},

												[Rvo2Line | Acc];
											false ->
												case DistSqCutoff =< DistSqRight of
													true ->
														case IsLeftLegForeign of
															true ->
																Acc;
															false ->
																Direction = LeftLegDirection4,
																Rvo2Line = #rvo2_line{direction = Direction, point = rvo2_vector2:add(LeftCutOff , rvo2_vector2:mult(Radius * InvTimeHorizonObst, rvo2_vector2:init(-Direction#rvo2_vector.y, Direction#rvo2_vector.x)))},

																[Rvo2Line | Acc]
														end;
													false ->
														case IsRightLegForeign of
															true ->
																Acc;
															false ->
																Direction = rvo2_vector2:negative(RightLegDirection4),
																Rvo2Line = #rvo2_line{direction = Direction, point = rvo2_vector2:add(RightCutOff , rvo2_vector2:mult(Radius * InvTimeHorizonObst, rvo2_vector2:init(-Direction#rvo2_vector.y, Direction#rvo2_vector.x)))},

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
		
		RelativePosition = rvo2_vector2:sub(Other#rvo2_agent.position, Position),
		RelativeVelocity = rvo2_vector2:sub(Velocity, Other#rvo2_agent.velocity),
		DistSq = rvo2_match:absSq(RelativePosition),
		CombinedRadius = Radius + Other#rvo2_agent.radius,
		CombinedRadiusSq = rvo2_match:sqr(CombinedRadius),

		Line = #rvo2_line{},

		% lager:info("id ~w DistSq ~w CombinedRadiusSq ~w RelativePosition : ~w Other#rvo2_agent.position ~w position ~w ~n", [Agent#rvo2_agent.id, DistSq, CombinedRadiusSq, RelativePosition, Other#rvo2_agent.position, Position]),

		{Direction2, U} = case DistSq > CombinedRadiusSq of
			true ->
				%% No collision. 
				W = rvo2_vector2:sub(RelativeVelocity, rvo2_vector2:mult(InvTimeHorizon2 , RelativePosition)),

				%% Vector from cutoff center to relative velocity.
				WLengthSq = rvo2_match:absSq(W),
				DotProduct1 = rvo2_vector2:mult(W, RelativePosition),

				% lager:info("id ~w DotProduct1 ~w rvo2_match:sqr(DotProduct1) ~w CombinedRadiusSq * WLengthSq ~w~n",[Agent#rvo2_agent.id, DotProduct1, rvo2_match:sqr(DotProduct1), CombinedRadiusSq * WLengthSq]),

				case DotProduct1 < 0.0 andalso rvo2_match:sqr(DotProduct1) > CombinedRadiusSq * WLengthSq of
					true ->
						%% Project on cut-off circle
						WLength = rvo2_match:sqrt(WLengthSq),
						UnitW 	= rvo2_vector2:divide(W, WLength),
						% lager:info("Direction ~w ~n",[rvo2_vector2:init(UnitW#rvo2_vector.y, -UnitW#rvo2_vector.x)]), 
						{rvo2_vector2:init(UnitW#rvo2_vector.y, -UnitW#rvo2_vector.x), rvo2_vector2:mult(CombinedRadius * InvTimeHorizon2 - WLength, UnitW)};
					false ->
						Leg = rvo2_match:sqrt(DistSq - CombinedRadiusSq),
						Direction = case rvo2_match:det(RelativePosition, W) > 0.0 of
							true ->
								%% Project on left leg.
								rvo2_vector2:divide(rvo2_vector2:init(RelativePosition#rvo2_vector.x * Leg - RelativePosition#rvo2_vector.y * CombinedRadius, RelativePosition#rvo2_vector.x * CombinedRadius + RelativePosition#rvo2_vector.y * Leg), DistSq);	
							false ->
								%% Project on right leg. 
								rvo2_vector2:divide(rvo2_vector2:negative(rvo2_vector2:init(RelativePosition#rvo2_vector.x * Leg + RelativePosition#rvo2_vector.y * CombinedRadius, -RelativePosition#rvo2_vector.x * CombinedRadius + RelativePosition#rvo2_vector.y * Leg)), DistSq)
						end,
						{Direction, rvo2_vector2:sub(rvo2_vector2:mult(rvo2_vector2:mult(RelativeVelocity, Direction), Direction), RelativeVelocity)}
				end;
			false ->
				%% Collision. Project on cut-off circle of time timeStep.

				InvTimeStep = 1.0 / TimeStep,
				
				%% Vector from cutoff center to relative velocity.

				W = rvo2_vector2:sub(RelativeVelocity, rvo2_vector2:mult(InvTimeStep , RelativePosition)),
				
				WLength = rvo2_match:abs(W),

				% lager:info(" id ~w  RelativeVelocity ~w InvTimeStep ~w  RelativePosition ~w w ~w WLength ~w~n",[Agent#rvo2_agent.id, RelativeVelocity, InvTimeStep, RelativePosition, W, WLength]),
				
				UnitW 	= rvo2_vector2:divide(W, WLength),

				Direction = rvo2_vector2:init(UnitW#rvo2_vector.y , - UnitW#rvo2_vector.x),

				% lager:info("Direction ~w ~n",[Direction]),

				{Direction, rvo2_vector2:mult(CombinedRadius * InvTimeStep - WLength, UnitW)}
		end,
		Line2 = Line#rvo2_line{direction = Direction2, point = rvo2_vector2:add(Velocity, rvo2_vector2:mult(0.5, U)) },
		[Line2 | Acc]
	end,
	OrcaLines3 = lists:foldl(FF, OrcaLines2, AgentNeighbors),

	% lager:info("OrcaLines3 ~w MaxSpeed ~w PrefVelocity ~w NewVelocity ~w~n",[OrcaLines3, MaxSpeed, PrefVelocity, NewVelocity]),

	{LineFail, Result} = linearProgram2(OrcaLines3, MaxSpeed, PrefVelocity, false, NewVelocity),
		
	% lager:info("id = ~w LineFail ~w, NewVelocity ~w ~n",[Agent#rvo2_agent.id, LineFail, Result]),

	NewVelocity2 = case LineFail =< length(OrcaLines3) of
		true ->
			% lager:info("========= ~w ~n",[Agent#rvo2_agent.id]),
			linearProgram3(OrcaLines3, Len, LineFail, MaxSpeed, Result);
		false ->
			% lager:info("====ddd===== ~w ~w ~n",[Agent#rvo2_agent.id, Result]),
			Result
	end,
	Agent#rvo2_agent{newVelocity = NewVelocity2, orcaLines = OrcaLines3}.


alreadyCovered([], _, _, _, _) -> false;
alreadyCovered([OrcaLine | T], InvTimeHorizonObst, RelativePosition1, RelativePosition2, Agent = #rvo2_agent{radius = Radius}) ->
	case
		rvo2_match:det(rvo2_vector2:sub(rvo2_vector2:mult(InvTimeHorizonObst, RelativePosition1), OrcaLine#rvo2_line.point), OrcaLine#rvo2_line.direction) - InvTimeHorizonObst * Radius >= -?RVO_EPSILON 
	andalso 
		rvo2_match:det(rvo2_vector2:sub(rvo2_vector2:mult(InvTimeHorizonObst, RelativePosition2), OrcaLine#rvo2_line.point), OrcaLine#rvo2_line.direction) - InvTimeHorizonObst * Radius >= -?RVO_EPSILON
	of
		true ->
			true;
		false ->
			alreadyCovered(T, InvTimeHorizonObst, RelativePosition1, RelativePosition2, Agent)
	end.

linearProgram1(Lines, LineNo, Radius, OptVelocity, DirectionOpt, Result) ->
	Line = #rvo2_line{point = Point, direction = Direction} = lists:nth(LineNo, Lines),
	DotProduct = rvo2_vector2:mult( Point, Direction),
	Discriminant = rvo2_match:sqr(DotProduct) + rvo2_match:sqr(Radius) -  rvo2_match:absSq(Point),
	case Discriminant < 0.0 of
		true ->
			%% Max speed circle fully invalidates line lineNo.
			{false, Result};
		false ->
			SqrtDiscriminant = rvo2_match:sqrt(Discriminant),
			TLeft = -DotProduct - SqrtDiscriminant,
			TRight = -DotProduct + SqrtDiscriminant,
			{Tag, TLeft2, TRight2} = do_linearProgram1(lists:seq(1, LineNo), Line, Lines, TLeft, TRight),
			case Tag of
				false ->
					{Tag, Result};
				true ->
					case DirectionOpt of
						true ->
							%%	Optimize direction.
							case rvo2_vector2:mult(OptVelocity, Direction) > 0.0  of
								true ->
									%% Take right extreme.
									{true, rvo2_vector2:add(Point, rvo2_vector2:mult(TRight2, Direction))};
								false ->
									%% Take left extreme.
									{true, rvo2_vector2:add(Point, rvo2_vector2:mult(TLeft2, Direction))}	
							end;
						false ->
							%% Optimize closest point.
							T = rvo2_vector2:mult(Direction, rvo2_vector2:sub(OptVelocity, Point)),
							if
							 	T < TLeft2 ->
							 		{true, rvo2_vector2:add(Point, rvo2_vector2:mult(TLeft2, Direction))};
								T > TRight2 ->
									{true, rvo2_vector2:add(Point, rvo2_vector2:mult(TRight2, Direction))};
								true ->
									{true, rvo2_vector2:add(Point, rvo2_vector2:mult(T, Direction))}
							end
					end
			end
	end.

do_linearProgram1([], _, _, TLeft, TRight) -> {true, TLeft, TRight};
do_linearProgram1([H | T], LineNo, Lines, TLeft, TRight) ->
	
	#rvo2_line{direction = Direction, point = Point} = lists:nth(H, Lines),

	Denominator = rvo2_match:det(LineNo#rvo2_line.direction, Direction),
	Mumerator = rvo2_match:det(Direction, rvo2_vector2:sub(LineNo#rvo2_line.point, Point)),

	case rvo2_match:fabs(Denominator) =< ?RVO_EPSILON of
		true ->
			case Mumerator < 0.0 of
				true ->
					{false, TLeft, TRight};	
				false ->
					do_linearProgram1(T, LineNo, Lines, TLeft, TRight)
			end;
		false ->
			Val = Mumerator / Denominator,

			{TLeft2, TRight2} = case Denominator >= 0.0 of
				true ->
					{TLeft, min(TRight, Val)};
				false ->
					{max(TLeft, Val), TRight}
			end,

			case TLeft2 > TRight2 of
				true ->
					{false, TLeft2, TRight2};
				false ->
					do_linearProgram1(T, LineNo, Lines, TLeft2, TRight2)
			end
	end.



linearProgram2(Lines, Radius, OptVelocity, DirectionOpt, _Result) ->
	Bool = rvo2_match:absSq(OptVelocity) > rvo2_match:sqr(Radius),
	
	Result2 = if
	 	DirectionOpt ->
	 		rvo2_vector2:mult(OptVelocity, Radius);
	 	Bool == true ->
	 		rvo2_vector2:mult(rvo2_match:normalize(OptVelocity), Radius);
		true ->
			OptVelocity
	end,
	
	{Count, Result3} = do_linearProgram2(lists:seq(1, length(Lines)), Lines, Radius, OptVelocity, DirectionOpt, Result2),

	case Count of
		false ->
			{length(Lines) + 1, Result3};
		_ ->
			{Count, Result3}
	end.

%% @spec do_linearProgram2(List, Lines, Radius, OptVelocity, DirectionOpt, Result) -> {false, NewResult} | {I, Result}
do_linearProgram2([], _, _, _, _, Result) -> {false, Result};
do_linearProgram2([H | T], Lines, Radius, OptVelocity, DirectionOpt, Result) ->
	Line = lists:nth(H, Lines),
	
	case rvo2_match:det(Line#rvo2_line.direction, rvo2_vector2:sub(Line#rvo2_line.point, Result)) > 0.0 of
		true ->
			{Tag, Result2} = linearProgram1(Lines, H, Radius, OptVelocity, DirectionOpt, Result),
			case not Tag of
				true ->
					% lager:info("Line ~w ~n",[Line]),
					{H, Result};
				false ->
					do_linearProgram2(T, Lines, Radius, OptVelocity, DirectionOpt, Result2)
			end;
		false ->
			do_linearProgram2(T, Lines, Radius, OptVelocity, DirectionOpt, Result)
	end.

linearProgram3(Lines, NumObstLines, BeginLine, Radius, Result) ->
	F = fun(I, {Distance, Result2}) ->
		#rvo2_line{direction = DirectionI, point = PointI} = lists:nth(I, Lines),
		case rvo2_match:det(DirectionI, rvo2_vector2:sub(PointI, Result2)) > Distance of
			true ->
				%%  Result does not satisfy constraint of line i.
				ProjLines = lists:sublist(Lines, 1, NumObstLines),

				FF = fun(J, AddProjLines) ->
					#rvo2_line{direction = DirectionJ, point = PointJ} = lists:nth(J, Lines),
					Determinant = rvo2_match:det(DirectionI, DirectionJ),

					case rvo2_match:fabs(Determinant) =< ?RVO_EPSILON of
						true ->
							case rvo2_vector2:mult(DirectionI, DirectionJ) > 0.0 of
								true ->
									AddProjLines;	
								false ->
									Line2 = #rvo2_line{point = rvo2_vector2:mult(0.5, rvo2_vector2:add(PointI, PointJ))},
									Line3 = Line2#rvo2_line{direction = rvo2_match:normalize(rvo2_vector2:sub(DirectionJ, DirectionI) ) },
									[Line3 | AddProjLines]
							end;
						false ->
							Line2 = #rvo2_line{point = rvo2_vector2:add(PointI, rvo2_vector2:mult( rvo2_match:det(DirectionJ, rvo2_vector2:sub(PointI, PointJ)) / Determinant, DirectionI))},

							Line3 = Line2#rvo2_line{direction = rvo2_match:normalize(rvo2_vector2:sub(DirectionJ, DirectionI) ) },
							[Line3 | AddProjLines]
					end
				end,

				AddProjLines = lists:foldl(FF, [], lists:seq(NumObstLines + 1, I)),
				ProjLines2 = ProjLines ++ lists:reverse(AddProjLines),
				% lager:info(" new vector2 ~w~n",[rvo2_vector2:init(-DirectionI#rvo2_vector.y, DirectionI#rvo2_vector.x)]),
				case linearProgram2(ProjLines2, Radius, rvo2_vector2:init(-DirectionI#rvo2_vector.y, DirectionI#rvo2_vector.x), true, Result2) of
					{Count, _Result3} when Count < length(ProjLines2) ->
						{rvo2_match:det(DirectionI, rvo2_vector2:sub(PointI, Result2)), Result2};
					{_, Result3} ->
						{rvo2_match:det(DirectionI, rvo2_vector2:sub(PointI, Result3)), Result3}
				end;
			false ->
				{Distance, Result2}
		end
	end,
	{_, Result2} = lists:foldl(F, {0, Result}, lists:seq(BeginLine, length(Lines))),
	Result2.

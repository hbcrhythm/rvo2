-module(rvo2_kd_tree).
-author('labihbc@gmail.com').

-include("rvo2.hrl").

-export([buildObstacleTree/2]).

buildAgentTree(Agents, KdTree = #rvo2_kd_tree{agents = Agents_}) ->
	case length(Agents_) == 0 orelse length(Agents_) != length(Agents) of
		true ->
			% KdTree#{agents = Agents},
			buildAgentTreeRecursive(KdTree);
		false ->
			KdTree
	end.

buildAgentTreeRecursive(KdTree) ->
	

buildObstacleTree(KdTree, Simulator = #rvo2_simulator{obstacles = Obstacles}) ->
 	ObstacleTree = buildObstacleTreeRecursive(Obstacles),
 	KdTree#rvo2_kd_tree{obstacleTree = ObstacleTree}.

buildObstacleTreeRecursive([]) ->
	undefined;
buildObstacleTreeRecursive(Obstacles) ->
	Len = length(Obstacles),

	Node = #rvo2_obstacle_tree_node{},

	OptimalSplit = 0,
	MinLeft = Len,
	MinRight = Len,
	
	F = fun(ObstacleI1 = #rvo2_obstacle{id = Id, next = ObstacleI2 = #rvo2_obstacle{point = ObstacleI2Point}, point = ObstacleI1Point}, {MinLeft2, MinRight2, OptimalSplit2}) ->
		LeftSize = 0,
		RightSize = 0,

		{LeftSize2, RightSize2} = buildObstacleTreeRecursive(lists:keydelete(Id, #rvo2_obstacle.id, Obstacles), ObstacleI1, LeftSize, RightSize, MinLeft2, MinRight2)

		FloatPair = rvo2_float_pair:init(max(LeftSize2, RightSize2), min(LeftSize2, RightSize2)),
		FloatPair2 = rvo2_float_pair:init(max(MinLeft, MinRight), min(MinLeft, MinRight)),

		case rvo2_float_pair:lt(FloatPair, FloatPair2) of
			true ->
				{LeftSize2, RightSize2, Id};
			false ->
				{MinLeft2, MinRight2, OptimalSplit2}
		end
	end,
	
	{MinLeft2, MinRight2, _, OptimalSplit2} = lists:foldl(F, {MinLeft, MinRight, 1, OptimalSplit}, Obstacle),

	%% Build Split Node

	LeftCounter = 0,
	RightCounter = 0,

	% LeftObstacles = [],
	% RightObstacles = [],

	I = OptimalSplit2,
	
	ObstacleI1 = #rvo2_obstacle{id = Id, next = ObstacleI2 = #rvo2_obstacle{point = ObstacleI2Point}, point = ObstacleI1Point} = lists:keyfind(Id, #rvo2_obstacle.id, Obstacles),

	F2 = fun(ObstacleJ1 = #rvo2_obstacle{next = ObstacleJ2 = #rvo2_obstacle{point = ObstacleJ2Point}, direction = Direction, point = ObstacleJ1Point}, {LeftObstacles, RightObstacles, AddObstacles}) ->
		J1LeftOfI = rvo2_match:leftOf(ObstacleI1Point, ObstacleI2Point, ObstacleJ1Point),
		J2LeftOfI = rvo2_match:leftOf(ObstacleI1Point, ObstacleI2Point, ObstacleJ2Point),

		if
		
			J1LeftOfI >= -?RVO_EPSILON andalso J2LeftOfI >= -?RVO_EPSILON ->
				{[ObstacleJ1 | LeftObstacles], RightObstacles, AddObstacles};
			J1LeftOfI =< ?RVO_EPSILON andalso J2LeftOfI =< ?RVO_EPSILON ->
				{LeftObstacles, [ObstacleJ1 | RightObstacles], AddObstacles};
			true ->
				T = rvo2_match:det( rvo2_vector2:subtract(ObstacleI2Point, ObstacleI1Point), rvo2_vector2:subtract(ObstacleJ2Point, ObstacleI1Point) ) / 
					rvo2_match:det( rvo2_vector2:subtract(ObstacleI2Point, ObstacleI1Point), rvo2_vector2:subtract(ObstacleJ1Point, ObstacleJ2Point) ) ,
				
				SplitPoint = rvo2_vector2:add(ObstacleJ1Point, rvo2_vector2:multiply(T, rvo2_vector2:subtract(ObstacleJ2Point, ObstacleJ1Point))),

				Rvo2Obstacle = #rvo2_obstacle{point = SplitPoint, previous = ObstacleJ1, next = ObstacleJ2, convex = true, direction = Direction, id = Len + length(AddObstacles) },

				AddObstacle2 = [Rvo2Obstacle | AddObstacles],

				NewObstacleJ1 = ObstacleJ1#rvo2_obstacle{next = Rvo2Obstacle},
				NewObstacleJ2 = ObstacleJ2#rvo2_obstacle{previous = Rvo2Obstacle},

				case J1LeftOfI > 0.0 of
					true ->
						{[NewObstacleJ1 | LeftObstacles], [Rvo2Obstacle | RightObstacles], AddObstacles};
					false ->
						{[Rvo2Obstacle | LeftObstacles], [NewObstacleJ1 | RightObstacles], AddObstacles};
				end
	end,
	{LeftObstacles, RightObstacles, AddObstacles} = lists:foldl(F2, {[], [], []}, lists:keydelete(I, #rvo2_obstacle.id, Obstacles),

	Node#rvo2_obstacle_tree_node{obstacle = ObstacleI1, left = buildObstacleTreeRecursive(LeftObstacles), right = buildObstacleTreeRecursive(RightObstacles)}.



buildObstacleTreeRecursive([], _, LeftSize, RightSize, _, _) -> {LeftSize, RightSize};
buildObstacleTreeRecursive([ObstacleJ1 = #rvo2_obstacle{next = ObstacleJ2 = #rvo2_obstacle{point = ObstacleJ2Point}, point = ObstacleJ1Point} | T], 
	ObstacleI1 = #rvo2_obstacle{id = Id, next = ObstacleI2 = #rvo2_obstacle{point = ObstacleI2Point}, point = ObstacleI1Point}, LeftSize, RightSize, MinLeft, MinRight) ->

	J1LeftOfI = rvo2_match:leftOf(ObstacleI1Point, ObstacleI2Point, ObstacleJ1Point),
	J2LeftOfI = rvo2_match:leftOf(ObstacleI1Point, ObstacleI2Point, ObstacleJ2Point),

	{LeftSize2, RightSize2} = if
		J1LeftOfI >= -?RVO_EPSILON andalso J2LeftOfI >= -?RVO_EPSILON ->
			{LeftSize + 1, RightSize};
		J1LeftOfI =< ?RVO_EPSILON andalso J2LeftOfI =< ?RVO_EPSILON ->
			{LeftSize, RightSize + 1};
		true ->
			{LeftSize + 1, RightSize + 1}
	end,

	FloatPair = rvo2_float_pair:init(max(LeftSize2, RightSize2), min(LeftSize2, RightSize2)),
	FloatPair2 = rvo2_float_pair:init(max(MinLeft, MinRight), min(MinLeft, MinRight)),

	case rvo2_float_pair:ge(FloatPair, FloatPair2) of
		true ->
			{LeftSize, RightSize};
		false ->
			buildObstacleTreeRecursive(T, ObstacleI1, LeftSize2, RightSize2, MinLeft, MinRight)
	end.

computeObstacleNeighbors(Rvo2Agent, RangeSq, KdTree) ->
	queryObstacleTreeRecursive(Rvo2Agent, RangeSq, KdTree).	


queryObstacleTreeRecursive(Rvo2Agent, _, #rvo2_kd_tree{obstacleTree = undefined}) -> Rvo2Agent;
queryObstacleTreeRecursive(Rvo2Agent = #rvo2_agent{position = Position}, RangeSq, KdTree = #rvo2_kd_tree{obstacleTree = Node = #rvo2_obstacle_tree_node{obstacle = Obstacle1 = #rvo2_obstacle{next = Obstacle2}, left = Left, right = Right } } ) ->
	
	AgentLeftOfLine = rvo2_match:leftOf(Obstacle1#rvo2_obstacle.point, Obstacle2#rvo2_obstacle.point, Position),

	queryObstacleTreeRecursive(Rvo2Agent, RangeSq, ?RVO2_IF(AgentLeftOfLine >= 0.0, Left, Right)).

	DistSqLine = rvo2_match:sqr(AgentLeftOfLine) / rvo2_match:absSq( rvo2_vector2:subtract( Obstacle2#rvo2_obstacle.point - Obstacle1#rvo2_obstacle.point ) ) ,

	case DistSqLine < RangeSq of
		true ->
			Rvo2Agent2 = case AgentLeftOfLine < 0.0 of
				true ->
					rvo2_agent:insertObstacleNeighbor(Obstacle1, RangeSq, Rvo2Agent);
				false ->
					Rvo2Agent
			end,
			queryObstacleTreeRecursive(Rvo2Agent2, RangeSq, ?RVO2_IF(AgentLeftOfLine >= 0.0, Left, Right));
		false ->
			Rvo2Agent
	end.


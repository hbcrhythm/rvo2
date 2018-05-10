-module(rvo2_kd_tree).
-author('labihbc@gmail.com').

-include("rvo2.hrl").

-export([buildObstacleTree/2, computeObstacleNeighbors/3]).

-define(MAX_LEAF_SIZE, 10).

buildAgentTree(Agents, KdTree = #rvo2_kd_tree{agents = Agents_}) ->
	case length(Agents_) == 0 orelse length(Agents_) != length(Agents) of
		true ->
			% KdTree#{agents = Agents},
			buildAgentTreeRecursive(1, length(Agents_), KdTree#{agents = Agents});
		false ->
			KdTree
	end.

buildAgentTreeRecursive(Begin, End, KdTree) ->
	buildAgentTreeRecursive(Begin, End, 1, KdTree).
buildAgentTreeRecursive(Begin, End, Curr, KdTree = #rvo2_kd_tree{agents = Agents, agentTree = AgentTree}) ->
	Agent = lists:nth(Curr, Agents),
	AgentTreeNode = #rvo2_agent_tree_node{
		begin 	= Begin 
		,end 	= End
		,minX 	= Agent#rvo2_agent.position#rvo2_vector.x
		,maxX 	= Agent#rvo2_agent.position#rvo2_vector.x
		,minY	= Agent#rvo2_agent.position#rvo2_vector.y
		,maxY 	= Agent#rvo2_agent.position#rvo2_vector.y
	},

	F = fun F2(End, End, AgentTreeNode2) -> AgentTreeNode2;
			F2(Curr2, End, AgentTreeNode2 = #rvo2_agent_tree_node{maxX = MaxX, minX = MinX, maxY = MaxY, minY = MinY}) ->
				Curr2Agent = lists:nth(Curr2, Agents),
				AgentTreeNode3 = AgentTreeNode2#rvo2_agent_tree_node{
					maxX  = max(MaxX, Curr2Agent#rvo2_agent.position#rvo2_vector.x)
					,minX = min(MinX, Curr2Agent#rvo2_agent.position#rvo2_vector.x)
					,maxY = max(MaxY, Curr2Agent#rvo2_agent.position#rvo2_vector.y)
					,minY = min(MinY, Curr2Agent#rvo2_agent.position#rvo2_vector.y)
				},
				F2(Curr2 + 1, End, AgentTreeNode3)
	end,
	AgentTreeNode2 = F(Beigin + 1, End, AgentTreeNode).
	case End - Begin > ?MAX_LEAF_SIZE of
		true ->
			IsVertical = AgentTreeNode2#rvo2_agent_tree_node.maxX - AgentTreeNode2#rvo2_agent_tree_node.minX > AgentTreeNode2#rvo2_agent_tree_node.maxY - AgentTreeNode2#rvo2_agent_tree_node.minY,
			SplitValue = 0.5 * ?RVO2_IF(IsVertical, AgentTreeNode2#rvo2_agent_tree_node.maxX + AgentTreeNode2#rvo2_agent_tree_node.minX, AgentTreeNode2#rvo2_agent_tree_node.maxY + AgentTreeNode2#rvo2_agent_tree_node#minY),
			
			F2 = fun F2(Left, Right, Agents2) ->
						F3 = fun F3(Left2, Right2) ->
								case Left2 < Right2 andalso ?RVO2_IF(IsVertical, (lists:nth(Left2, Agents2))#rvo2_agent.position#rvo2_vector.x, (lists:nth(Left2, Agents2))#rvo2_agent.position#rvo2_vector.y) < SplitValue of
									true ->
										F3(Left2 + 1, Right2);
									false ->
										Left2
								end
						end,

						Left2 = F3(Left, Right),

						F4 = fun F4(Left3, Right3) ->
									case Right3 > Left3 andalso ?RVO2_IF(IsVertical, (lists:nth(Right3 - 1, Agents2))#rvo2_agent.position#rvo2_vector.x, (lists:nth(Right3 - 1, Agents2))#rvo2_agent.position#rvo2_vector.y) < SplitValue of
										true ->
											F4(Left3, Right3 - 1);
										false ->
											Right3
									end
						end,

						Right2 = F4(Left2, Right),

						case Left2 < Right2 of
							true ->
								LeftTempAgent = lists:nth(Left2, Agents),
								RightTempAgent = lists:nth(Right2, Agents),
								Agents2 = lists:keyreplace(LeftTempAgent#rvo2_agent.id, #rvo2_agent.id, Agents, RightTempAgent),
								Agents3 = lists:keyreplace(RightTempAgent#rvo2_agent.id, #rvo2_agent.id, Agents2, LeftTempAgent),
								F2(Left2 + 1, Right2 - 1, Agents3);
							false ->
								F2(Left2, Right2, Agents)
						end

			end,

			{Left3, Right3, Agent4} = F2(Begin, End, Agents),

			LeftSize = Left3 - Right3,

			{Left4, Right4, LeftSize2} = case LeftSize of
				0 ->
					{Left3 + 1, Right3 + 1, LeftSize + 1};
				false ->
					{Left3, Right3, LeftSize}
			end,


			AgentTreeNode3 = AgentTreeNode2#rvo2_agent_tree_node{left = Curr + 1 , right = Curr + 2 - LeftSize2},
			KdTree2 = KdTree#rvo2_kd_tree{agentTree = [AgentTreeNode3 | AgentTree],
		 	KdTree3 = buildAgentTreeRecursive(Begin, Left4, Curr + 1, KdTree2),
		 	KdTree4 = buildAgentTreeRecursive(Begin, Left4, KdTree2#rvo2_kd_tree{agentTree = [AgentTreeNode3 | AgentTree]}),
		 	KdTree4;
		false ->
			KdTree2 = KdTree#rvo2_kd_tree{agentTree = [AgentTreeNode3 | AgentTree],
			KdTree2
	end.


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

%% @spec computeObstacleNeighbors(Rvo2Agent, RangeSq, KdTree) -> NewRvo2Agent
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

computeAgentNeighbors(Agent, RangeSq, KdTree) ->
	queryAgentTreeRecursive(Agent, RangeSq, KdTree).

queryAgentTreeRecursive(Node, Agent, RangeSq, KdTree = #rvo2_kd_tree{agentTree = AgentTree}) ->
	AgentTreeNode = lists:nth(Node, AgentTree),
	case AgentTreeNode#rvo2_agent_tree_node.end - AgentTreeNode#rvo2_agent_tree_node.begin =< ?MAX_LEAF_SIZE of
		true ->
			F = fun F(Curr, End, RangeSq2, Agent2) ->
						Agent3 = lists:nth(Curr, AgentTree),
						{RangeSq3, Agent4} = rvo2_agent:insertAgentNeighbor(Agent3, RangeSq2, Agent2),
						F(Curr2 + 1, End, RangeSq3, Agent4);
			end,

			F( AgentTreeNode#rvo2_agent_tree_node.begin,  AgentTreeNode#rvo2_agent_tree_node.end, RangeSq, Agent)

		false ->

	end.

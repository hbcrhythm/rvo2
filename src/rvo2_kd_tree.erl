-module(rvo2_kd_tree).
-author('labihbc@gmail.com').

-include("rvo2.hrl").

-export([buildAgentTree/2, buildObstacleTree/2, computeObstacleNeighbors/3, computeAgentNeighbors/3, queryVisibility/4]).

-define(MAX_LEAF_SIZE, 10).

buildAgentTree(Agents, KdTree = #rvo2_kd_tree{agents = Agents_}) ->
	case length(Agents_) == 0 orelse length(Agents_) =/= length(Agents) of
		true ->
			buildAgentTreeRecursive(1, length(Agents), KdTree#rvo2_kd_tree{agents = Agents});
		false ->
			KdTree
	end.

buildAgentTreeRecursive(Begin, End, KdTree) ->
	buildAgentTreeRecursive(Begin, End, Begin, KdTree).
buildAgentTreeRecursive(Begin, End, Curr, KdTree = #rvo2_kd_tree{agents = Agents, agentTree = AgentTree}) ->
	Agent = lists:nth(Curr, Agents),
	AgentTreeNode = #rvo2_agent_tree_node{
		begin_ 	= Begin 
		,end_ 	= End
		,minX 	= Agent#rvo2_agent.position#rvo2_vector.x
		,maxX 	= Agent#rvo2_agent.position#rvo2_vector.x
		,minY	= Agent#rvo2_agent.position#rvo2_vector.y
		,maxY 	= Agent#rvo2_agent.position#rvo2_vector.y
	},

	F = fun F(Curr2, FEnd, AgentTreeNode2) when Curr2 > FEnd -> AgentTreeNode2;
			F(Curr2, FEnd, AgentTreeNode2 = #rvo2_agent_tree_node{maxX = MaxX, minX = MinX, maxY = MaxY, minY = MinY}) ->
				Curr2Agent = lists:nth(Curr2, Agents),
				AgentTreeNode3 = AgentTreeNode2#rvo2_agent_tree_node{
					maxX  = max(MaxX, Curr2Agent#rvo2_agent.position#rvo2_vector.x)
					,minX = min(MinX, Curr2Agent#rvo2_agent.position#rvo2_vector.x)
					,maxY = max(MaxY, Curr2Agent#rvo2_agent.position#rvo2_vector.y)
					,minY = min(MinY, Curr2Agent#rvo2_agent.position#rvo2_vector.y)
				},
				F(Curr2 + 1, FEnd, AgentTreeNode3)
	end,
	AgentTreeNode2 = F(Begin + 1, End, AgentTreeNode),
	case End - Begin > ?MAX_LEAF_SIZE of
		true ->
			
			%% No leaf node.

			IsVertical = AgentTreeNode2#rvo2_agent_tree_node.maxX - AgentTreeNode2#rvo2_agent_tree_node.minX > AgentTreeNode2#rvo2_agent_tree_node.maxY - AgentTreeNode2#rvo2_agent_tree_node.minY,
			SplitValue = 0.5 * ?RVO2_IF(IsVertical, AgentTreeNode2#rvo2_agent_tree_node.maxX + AgentTreeNode2#rvo2_agent_tree_node.minX, AgentTreeNode2#rvo2_agent_tree_node.maxY + AgentTreeNode2#rvo2_agent_tree_node.minY),
			
			F2 = fun 	F2(Left, Right, Agents2) when Left > Right -> 
							{Left, Right, Agents2};
						F2(Left, Right, Agents2) ->

							F3 = fun F3(Left2) ->
									case Left2 =< Right andalso ?RVO2_IF(IsVertical, (lists:nth(Left2, Agents2))#rvo2_agent.position#rvo2_vector.x, (lists:nth(Left2, Agents2))#rvo2_agent.position#rvo2_vector.y) < SplitValue of
										true ->
											F3(Left2 + 1);
										false ->
											Left2
									end
							end,

							Left2 = F3(Left),

							F4 = fun F4(Right2) ->
										case Right2 >= Left2 andalso ?RVO2_IF(IsVertical, (lists:nth(Right2, Agents2))#rvo2_agent.position#rvo2_vector.x, (lists:nth(Right2, Agents2))#rvo2_agent.position#rvo2_vector.y) < SplitValue of
											true ->
												F4(Right2 - 1);
											false ->
												Right2
										end
							end,

							Right2 = F4(Right),

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

			{Left3, Right3, Agents2} = F2(Begin, End, Agents),

			LeftSize = Left3 - Right3,

			{Left4, _Right4, LeftSize2} = case LeftSize of
				0 ->
					{Left3 + 1, Right3 + 1, LeftSize + 1};
				false ->
					{Left3, Right3, LeftSize}
			end,

			AgentTreeNode3 = AgentTreeNode2#rvo2_agent_tree_node{left = Curr + 1 , right = Curr + 2 * LeftSize2},
			KdTree2 = KdTree#rvo2_kd_tree{agentTree = [AgentTreeNode3 | AgentTree], agents = Agents2},

		 	KdTree3 = buildAgentTreeRecursive(Begin, Left4, Curr + 1, KdTree2),

		 	KdTree4 = buildAgentTreeRecursive(Left4, End, KdTree3),

		 	KdTree4;
		false ->
			KdTree2 = KdTree#rvo2_kd_tree{agentTree = [AgentTreeNode2 | AgentTree]},
			KdTree2
	end.

%% @spec buildObstacleTree(KdTree, Simulator) -> NewSimulator
%% @doc  Builds an obstacle k-D tree.
buildObstacleTree(KdTree, Simulator = #rvo2_simulator{obstacles = Obstacles}) ->
 	{ObstacleTree, Obstacles2} = buildObstacleTreeRecursive(Obstacles, Obstacles),
 	KdTree2 = KdTree#rvo2_kd_tree{obstacleTree = ObstacleTree, obstacles = Obstacles2},
 	Simulator#rvo2_simulator{kdTree = KdTree2, obstacles = Obstacles2}.

%% @spec buildObstacleTreeRecursive(Obstacles) -> {ObstacleTree, Obstacles2}
%% @doc Recursive method for building an obstacle k-D tree.
buildObstacleTreeRecursive([], Obstacles) -> {undefined, Obstacles};
buildObstacleTreeRecursive(Obstacles, ObstaclesAcc) ->
	Len = length(Obstacles),

	Node = #rvo2_obstacle_tree_node{},

	% OptimalSplit = 0,

	MinLeft = Len,
	MinRight = Len,
	
	
	F = fun(ObstacleI1 = #rvo2_obstacle{id = Id}, {MinLeft2, MinRight2, OptimalSplit}) ->
		LeftSize = 0,
		RightSize = 0,

		{LeftSize2, RightSize2} = buildObstacleTreeRecursive(Obstacles, ObstaclesAcc, ObstacleI1, LeftSize, RightSize, MinLeft2, MinRight2),

		FloatPair = rvo2_float_pair:init(max(LeftSize2, RightSize2), min(LeftSize2, RightSize2)),
		FloatPair2 = rvo2_float_pair:init(max(MinLeft, MinRight), min(MinLeft, MinRight)),

		case rvo2_float_pair:lt(FloatPair, FloatPair2) of
			true ->
				{LeftSize2, RightSize2, Id};
			false ->
				{MinLeft2, MinRight2, OptimalSplit}
		end
	end,
	
	{_MinLeft2, _MinRight2, _, OptimalSplit} = lists:foldl(F, {MinLeft, MinRight, 1}, Obstacles),

	%% Build Split Node

	I = OptimalSplit,
	
	ObstacleI1 = #rvo2_obstacle{next_id = NextIId, point = ObstacleI1Point} = lists:keyfind(I, #rvo2_obstacle.id, ObstaclesAcc),
	_ObstacleI2 = #rvo2_obstacle{point = ObstacleI2Point} = lists:keyfind(NextIId, #rvo2_obstacle.id, ObstaclesAcc),


	F2 = fun(ObstacleJ1 = #rvo2_obstacle{id = J1Id, next_id = NextJId, direction = J1Direction, point = ObstacleJ1Point}, {LeftObstacles, RightObstacles, AddObstacles, Obstacles2}) ->

		ObstacleJ2 = #rvo2_obstacle{id = J2Id, point = ObstacleJ2Point} = lists:keyfind(NextJId, #rvo2_obstacle.id, Obstacles2),

		J1LeftOfI = rvo2_match:leftOf(ObstacleI1Point, ObstacleI2Point, ObstacleJ1Point),
		J2LeftOfI = rvo2_match:leftOf(ObstacleI1Point, ObstacleI2Point, ObstacleJ2Point),

		if
		
			J1LeftOfI >= -?RVO_EPSILON andalso J2LeftOfI >= -?RVO_EPSILON ->
				{[ObstacleJ1 | LeftObstacles], RightObstacles, AddObstacles, Obstacles2};
			J1LeftOfI =< ?RVO_EPSILON andalso J2LeftOfI =< ?RVO_EPSILON ->
				{LeftObstacles, [ObstacleJ1 | RightObstacles], AddObstacles, Obstacles2};
			true ->
				T = rvo2_match:det(rvo2_vector2:subtract(ObstacleI2Point, ObstacleI1Point), rvo2_vector2:subtract(ObstacleJ1Point, ObstacleI1Point) ) / 
					rvo2_match:det(rvo2_vector2:subtract(ObstacleI2Point, ObstacleI1Point), rvo2_vector2:subtract(ObstacleJ1Point, ObstacleJ2Point) ) ,
				
				SplitPoint = rvo2_vector2:add(ObstacleJ1Point, rvo2_vector2:multiply(T, rvo2_vector2:subtract(ObstacleJ2Point, ObstacleJ1Point))),

				NewId =  Len + length(AddObstacles),
				NewObstacle = #rvo2_obstacle{point = SplitPoint, previous_id = J1Id, next_id = J2Id, convex = true, direction = J1Direction, id = NewId},

				AddObstacle2 = [NewObstacle | AddObstacles],

				NewObstacleJ1 = ObstacleJ1#rvo2_obstacle{next_id = NewId},
				NewObstacleJ2 = ObstacleJ2#rvo2_obstacle{previous_id = NewId},

				Obstacles3 = lists:keyreplace(J1Id, #rvo2_obstacle.id, Obstacles2, NewObstacleJ1),
				Obstacles4 = lists:keyreplace(J2Id, #rvo2_obstacle.id, Obstacles3, NewObstacleJ2),

				case J1LeftOfI > 0.0 of
					true ->
						{[NewObstacleJ1 | LeftObstacles], [NewObstacle | RightObstacles], AddObstacle2, Obstacles4};
					false ->
						{[NewObstacle | LeftObstacles], [NewObstacleJ1 | RightObstacles], AddObstacle2, Obstacles4}
				end
		end
	end,
	{LeftObstacles, RightObstacles, AddObstacles, Obstacles2} = lists:foldl(F2, {[], [], [], ObstaclesAcc}, lists:keydelete(I, #rvo2_obstacle.id, Obstacles)),

	Obstacles3 = lists:keysort(#rvo2_obstacle.id, AddObstacles ++ Obstacles2),

	{Node2, Obstacles4} = buildObstacleTreeRecursive(LeftObstacles, Obstacles3),
	{Node3, Obstacles5}	= buildObstacleTreeRecursive(RightObstacles, Obstacles4),

	{Node#rvo2_obstacle_tree_node{obstacle = ObstacleI1, left = Node2, right = Node3}, Obstacles5}.


%% Compute optimal split node.
buildObstacleTreeRecursive(Obstacles, ObstaclesAcc, ObstacleI1 = #rvo2_obstacle{id = Id}, LeftSize, RightSize, MinLeft, MinRight) ->
	Obstacles2 = lists:keydelete(Id, #rvo2_obstacle.id, Obstacles),
	buildObstacleTreeRecursive2(Obstacles2, ObstaclesAcc, ObstacleI1, LeftSize, RightSize, MinLeft, MinRight).

buildObstacleTreeRecursive2([], _, _, LeftSize, RightSize, _MinLeft, _MinRight) -> {LeftSize, RightSize};
buildObstacleTreeRecursive2([_ObstacleJ1 = #rvo2_obstacle{point = ObstacleJ1Point, next_id = NextJId} | T], Obstacles, ObstacleI1 = #rvo2_obstacle{next_id = NextIId, point = ObstacleI1Point}, LeftSize, RightSize, MinLeft, MinRight) ->
	_ObstacleI2 = #rvo2_obstacle{point = ObstacleI2Point} = lists:keyfind(NextIId, #rvo2_obstacle.id, Obstacles),
	_ObstacleJ2 = #rvo2_obstacle{point = ObstacleJ2Point} = lists:keyfind(NextJId, #rvo2_obstacle.id, Obstacles),

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
			{LeftSize2, RightSize2};
		false ->
			buildObstacleTreeRecursive2(T, Obstacles, ObstacleI1, LeftSize2, RightSize2, MinLeft, MinRight)
	end.

%% @spec computeObstacleNeighbors(Agent, RangeSq, KdTree) -> NewAgent
computeObstacleNeighbors(Agent, RangeSq, #rvo2_kd_tree{obstacleTree = ObstacleTree, obstacles = Obstacles}) ->
	queryObstacleTreeRecursive(Agent, RangeSq, Obstacles, ObstacleTree).	

queryObstacleTreeRecursive(Agent, _, _, #rvo2_kd_tree{obstacleTree = undefined}) -> Agent;
queryObstacleTreeRecursive(Agent = #rvo2_agent{position = Position}, RangeSq, Obstacles, #rvo2_obstacle_tree_node{obstacle = Obstacle1 = #rvo2_obstacle{next_id = NextId}, left = Left, right = Right}) ->
	
	Obstacle2 = lists:keyfind(NextId, #rvo2_obstacle.id, Obstacles),

	AgentLeftOfLine = rvo2_match:leftOf(Obstacle1#rvo2_obstacle.point, Obstacle2#rvo2_obstacle.point, Position),

	Agent2 = queryObstacleTreeRecursive(Agent, RangeSq, Obstacles, ?RVO2_IF(AgentLeftOfLine >= 0.0, Left, Right)),

	DistSqLine = rvo2_match:sqr(AgentLeftOfLine) / rvo2_match:absSq(rvo2_vector2:subtract(Obstacle2#rvo2_obstacle.point - Obstacle1#rvo2_obstacle.point)) ,

	case DistSqLine < RangeSq of
		true ->
			Agent3 = case AgentLeftOfLine < 0.0 of
				true ->
					rvo2_agent:insertObstacleNeighbor(Obstacle1, Obstacle2, RangeSq, Agent2);
				false ->
					Agent2
			end,
			queryObstacleTreeRecursive(Agent3, RangeSq, Obstacles, ?RVO2_IF(AgentLeftOfLine >= 0.0, Left, Right));
		false ->
			Agent2
	end.

%% @doc computeAgentNeighbors(Agent, RangeSq, KdTree) -> {RangeSq2, Agent2}.
computeAgentNeighbors(Agent, RangeSq, KdTree) ->
	queryAgentTreeRecursive(1, Agent, RangeSq, KdTree).

queryAgentTreeRecursive(Node, Agent, RangeSq, KdTree = #rvo2_kd_tree{agentTree = AgentTree, agents = Agents}) ->
	
	AgentTreeNode = #rvo2_agent_tree_node{left = Left, right = Right} = lists:nth(Node, AgentTree),

	case AgentTreeNode#rvo2_agent_tree_node.end_ - AgentTreeNode#rvo2_agent_tree_node.begin_ =< ?MAX_LEAF_SIZE of
		true ->
			F = fun F(Curr, End, RangeSq2, Agent2) when Curr >= End ->
						{RangeSq2, Agent2};
					F(Curr, End, RangeSq2, Agent2) ->
						Agent3 = lists:nth(Curr, Agents),
						{RangeSq3, Agent4} = rvo2_agent:insertAgentNeighbor(Agent3, RangeSq2, Agent2),
						F(Curr + 1, End, RangeSq3, Agent4)
			end,

			{RangeSq2, Agent2} = F(AgentTreeNode#rvo2_agent_tree_node.begin_, AgentTreeNode#rvo2_agent_tree_node.end_, RangeSq, Agent),
			{RangeSq2, Agent2};
		false ->
			LeftAgentTreeNode = lists:nth(Left, AgentTree),
			RightAgentTreeNode = lists:nth(Right, AgentTree),

			DistSqLeft = rvo2_match:sqr( max(0.0, LeftAgentTreeNode#rvo2_agent_tree_node.minX - Agent#rvo2_agent.position#rvo2_vector.x)) + 
						 rvo2_match:sqr( max(0.0, Agent#rvo2_agent.position#rvo2_vector.x - LeftAgentTreeNode#rvo2_agent_tree_node.maxX)) + 
						 rvo2_match:sqr( max(0.0, LeftAgentTreeNode#rvo2_agent_tree_node.minY - Agent#rvo2_agent.position#rvo2_vector.y)) + 
						 rvo2_match:sqr( max(0.0, Agent#rvo2_agent.position#rvo2_vector.y - LeftAgentTreeNode#rvo2_agent_tree_node.maxY)),

			DistSqRight= rvo2_match:sqr( max(0.0, RightAgentTreeNode#rvo2_agent_tree_node.minX - Agent#rvo2_agent.position#rvo2_vector.x)) + 
						 rvo2_match:sqr( max(0.0, Agent#rvo2_agent.position#rvo2_vector.x - RightAgentTreeNode#rvo2_agent_tree_node.maxX)) + 
						 rvo2_match:sqr( max(0.0, RightAgentTreeNode#rvo2_agent_tree_node.minY - Agent#rvo2_agent.position#rvo2_vector.y)) + 
						 rvo2_match:sqr( max(0.0, Agent#rvo2_agent.position#rvo2_vector.y - RightAgentTreeNode#rvo2_agent_tree_node.maxY)),

			case DistSqLeft < DistSqRight of
				true ->
					case DistSqLeft < RangeSq of
						true ->
							{RangeSq2, Agent2} = queryAgentTreeRecursive(Left, Agent, RangeSq, KdTree),
							case DistSqRight < RangeSq2 of
								true ->
									queryAgentTreeRecursive(Right, Agent2, RangeSq2, KdTree);
								false ->
									{RangeSq2, Agent2}
							end;
						false ->
							{RangeSq, Agent}
					end;
				false ->
					case DistSqRight < RangeSq of
						true ->
							{RangeSq2, Agent2} = queryAgentTreeRecursive(Right, Agent, RangeSq, KdTree),
							case DistSqLeft < RangeSq2 of
								true ->
									queryAgentTreeRecursive(Left, Agent2, RangeSq2, KdTree);
								false ->
									{RangeSq2, Agent2}
							end;
						false ->
							{RangeSq, Agent}
					end
			end
	end.

queryVisibility(Q1, Q2, Radius, #rvo2_kd_tree{obstacleTree = ObstacleTree, obstacles = Obstacles}) ->
	queryVisibilityRecursive(Q1, Q2, Radius, Obstacles, ObstacleTree).

queryVisibilityRecursive(_, _, _, _, undefined) ->	true;
queryVisibilityRecursive(Q1, Q2, Radius, Obstacles, #rvo2_obstacle_tree_node{obstacle = Obstacle1 = #rvo2_obstacle{next_id = NextId}, left = Left, right = Right}) ->

	Obstacle2 = lists:keyfind(NextId, #rvo2_obstacle.id, Obstacles),

	Q1LeftOfI = rvo2_match:leftOf(Obstacle1#rvo2_obstacle.point, Obstacle2#rvo2_obstacle.point, Q1),
	Q2LeftOfI = rvo2_match:leftOf(Obstacle1#rvo2_obstacle.point, Obstacle2#rvo2_obstacle.point, Q2),

	InvLengthI = 1.0 / rvo2_match:absSq(rvo2_vector2:subtract(Obstacle2#rvo2_obstacle.point, Obstacle1#rvo2_obstacle.point)),

	case Q1LeftOfI >= 0.0 andalso Q2LeftOfI >= 0.0 of
		true ->
			queryVisibilityRecursive(Q1, Q2, Radius, Obstacles, Left) andalso 
			rvo2_match:sqr(Q1LeftOfI) * InvLengthI >= rvo2_match:sqr(Radius) andalso rvo2_match:sqr(Q2LeftOfI) * InvLengthI >= rvo2_match:sqr(Radius) orelse
			queryVisibilityRecursive(Q1, Q2, Radius, Obstacles, Right);
		false ->
			case Q1LeftOfI =< 0.0 andalso Q2LeftOfI =< 0.0 of
				true ->
					queryVisibilityRecursive(Q1, Q2, Radius, Obstacles, Right) andalso 
					rvo2_match:sqr(Q1LeftOfI) * InvLengthI >= rvo2_match:sqr(Radius) andalso rvo2_match:sqr(Q2LeftOfI) * InvLengthI >= rvo2_match:sqr(Radius) orelse
					queryVisibilityRecursive(Q1, Q2, Radius, Obstacles, Left);
				false ->
					case Q1LeftOfI >= 0.0 andalso Q2LeftOfI =< 0.0 of
						true ->
							queryVisibilityRecursive(Q1, Q2, Radius, Obstacles, Left) andalso 
							queryVisibilityRecursive(Q1, Q2, Radius, Obstacles, Right);
						false ->
							Point1LeftOfQ = rvo2_match:leftOf(Q1, Q2, Obstacle1#rvo2_obstacle.point),
							Point2LeftOfQ = rvo2_match:leftOf(Q1, Q2, Obstacle2#rvo2_obstacle.point),
							InvLengthQ = 1.0 / rvo2_match:absSq(rvo2_vector2:subtract(Q2, Q1)),
							
							Point1LeftOfQ * Point2LeftOfQ >= 0.0 andalso 
							rvo2_match:sqr(Point1LeftOfQ) * InvLengthQ > rvo2_match:sqr(Radius) andalso
							rvo2_match:sqr(Point2LeftOfQ) * InvLengthQ > rvo2_match:sqr(Radius) andalso
							queryVisibilityRecursive(Q1, Q2, Radius, Obstacles, Left) andalso 
							queryVisibilityRecursive(Q1, Q2, Radius, Obstacles, Right)
					end
			end
	end.


	
% queryAgentTreeRecursive(Node, Agent, RangeSq, Position)
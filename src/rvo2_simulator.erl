-module(rvo2_simulator).
-author('labihbc@gmail.com').

-include("rvo2.hrl").

-define(DEFAULT_NUM_WORKER, 4).

-export([init/0, setTimeStep/2, setAgentDefaults/8, processObstacles/1, addAgent/2, addObstacle/2, doStep/1, getNumAgents/1]).


init() ->
	Simulator = #rvo2_simulator{},
	Simulator2 = setNumWorkers(0, Simulator),
	Simulator2.

setTimeStep(TimeStep, Simulator) ->
	Simulator#rvo2_simulator{timeStep = TimeStep}.

setAgentDefaults(NeighborDist, MaxNeighbors, TimeHorizon, TimeHorizonObst, Radius, MaxSpeed, Velocity, Simulator) ->
	DefaultAgent = #rvo2_agent{
		maxNeighbors 		= MaxNeighbors
		,maxSpeed 			= MaxSpeed
		,neighborDist 		= NeighborDist
		,radius 			= Radius
		,timeHorizon 		= TimeHorizon
		,timeHorizonObst 	= TimeHorizonObst
		,velocity 			= Velocity
	},
	Simulator#rvo2_simulator{defaultAgent = DefaultAgent}.


setNumWorkers(NumWorkers, Simulator) ->
	case NumWorkers =< 0 of
		true ->
			Simulator#rvo2_simulator{numWorkers = ?DEFAULT_NUM_WORKER, workers = [], workerAgentCount = 0};
		false ->
			Simulator#rvo2_simulator{numWorkers = NumWorkers, workers = [], workerAgentCount = 0}
	end.

processObstacles(Simulator = #rvo2_simulator{kdTree = KdTree}) ->
	KdTree2 = rvo2_kd_tree:buildObstacleTree(KdTree, Simulator),
	Simulator#rvo2_simulator{kdTree = KdTree2}.

addObstacle(Vertices, Simulator = #rvo2_simulator{obstacles = Obstacles}) ->
	LenVertices = length(Vertices),
	case LenVertices < 2 of
		true ->
			Simulator;
		false ->

			InitLen = length(Obstacles),

			F = fun F([], Obstacles2) -> Obstacles2;
					F([VerticeId | T], Obstacles2) ->
					
					Id = length(Obstacles2) + 1,

					Vertice = lists:nth(VerticeId, Vertices),
					Obstacle	= #rvo2_obstacle{id = Id, point = Vertice},

					{Obstacle3, Obstacles4} = case VerticeId =/= 1 of
						true ->

							Previous = #rvo2_obstacle{id = PreviousId}  = lists:keyfind(length(Obstacles2), #rvo2_obstacle.id, Obstacles2),

							Previous2 = Previous#rvo2_obstacle{next_id = Id},

							Obstacle2 = Obstacle#rvo2_obstacle{previous_id = PreviousId},

							Obstacles3 = lists:keyreplace(PreviousId, #rvo2_obstacle.id, Obstacle2, Previous2),
							{Obstacle2, Obstacles3};
						false ->

							{Obstacle, Obstacles2}
					end,

					{Obstacle5, Obstacles6} = case VerticeId == LenVertices of
						true ->
							ObstacleNoId = ?RVO2_IF(InitLen == 0, 1, InitLen),

							Obstacle4 = Obstacle3#rvo2_obstacle{next_id = ObstacleNoId},

							Next = lists:keyfind(ObstacleNoId, #rvo2_obstacle.id, Obstacles4),
							
							Next2 = Next#rvo2_obstacle{previous_id = ObstacleNoId},

							Obstacles5 = lists:keyreplace(ObstacleNoId, #rvo2_obstacle.id, Obstacles4, Next2),

							{Obstacle4, Obstacles5};
						false ->
							{Obstacle3, Obstacles4}
					end,

					Obstacle6 = Obstacle5#rvo2_obstacle{direction = rvo2_match:normalize(rvo2_vector:subtract(?RVO2_IF(VerticeId ==  LenVertices, lists:nth(1, Vertices), lists:nth(VerticeId + 1, Vertices)) - lists:nth(VerticeId, Vertices)))},

					Obstacle7 = case LenVertices of
						2 ->
							Obstacle6#rvo2_obstacle{convex = true};
						_ ->
							Convex = (rvo2_match:leftOf(?RVO2_IF(VerticeId == 1, lists:nth(LenVertices, Vertices), lists:nth(VerticeId - 1, Vertices)), lists:nth(VerticeId, Vertices), lists:nth( ?RVO2_IF(VerticeId == LenVertices, 1, VerticeId + 1) )) >= 0.0),
							Obstacle6#rvo2_obstacle{convex = Convex}
					end,
					F(T, [Obstacle7 | Obstacles6])

			end,

			Obstacles2 = F(lists:seq(1, LenVertices), Obstacles),
			Obstacles3 = lists:keysort(#rvo2_obstacle.id, Obstacles2),
			Simulator#rvo2_simulator{obstacles = Obstacles3}
	end.


addAgent(_Rvo2Vector2, Simulator = #rvo2_simulator{defaultAgent = undefined}) ->
	{-1, Simulator};
addAgent(Rvo2Vector2, Simulator = #rvo2_simulator{defaultAgent = DefaultAgent, s_totalID = STotalId, agents = Agents}) ->
	Agent = #rvo2_agent{
		id 				= STotalId
		,maxNeighbors 	= DefaultAgent#rvo2_agent.maxNeighbors
		,maxSpeed 		= DefaultAgent#rvo2_agent.maxSpeed
		,neighborDist 	= DefaultAgent#rvo2_agent.neighborDist
		,position 		= Rvo2Vector2
		,radius 		= DefaultAgent#rvo2_agent.radius
		,timeHorizon 	= DefaultAgent#rvo2_agent.timeHorizon
		,timeHorizonObst= DefaultAgent#rvo2_agent.timeHorizonObst
		,velocity 		= DefaultAgent#rvo2_agent.velocity
	},
	
	STotalId2 = STotalId + 1,

	Agents2 = lists:reverse([Agent | Agents]),

	Simulator2 = Simulator#rvo2_simulator{s_totalID = STotalId2, agents = Agents2},

	Simulator3 = onAddAgent(Simulator2),

	{STotalId2, Simulator3}.

onAddAgent(Simulator = #rvo2_simulator{agents = []}) -> Simulator;
onAddAgent(Simulator = #rvo2_simulator{agents = Agents, agentNo2indexDict = AgentNo2indexDict, index2agentNoDict = Index2agentNoDict}) ->
	Index = length(Agents),
	AgentNo = (lists:nth(Index, Agents))#rvo2_agent.id,
	AgentNo2indexDict2 = dict:append(AgentNo, Index, AgentNo2indexDict),
	Index2agentNoDict2 = dict:append(Index, AgentNo, Index2agentNoDict),

	Simulator#rvo2_simulator{agentNo2indexDict = AgentNo2indexDict2, index2agentNoDict = Index2agentNoDict2}.

doStep(Simulator) ->
	Simulator2 = #rvo2_simulator{workers = Workers, numWorkers = NumWorkers, workerAgentCount = WorkerAgentCount, kdTree = KdTree, agents = Agents, globalTime = GlobalTime, timeStep = TimeStep} = updateDeleteAgent(Simulator),
	
	Length = getNumAgents(Simulator2),

	Workers2 = case Workers of
		[] ->
			[rvo2_worker:init( trunc((Index - 1) * Length / NumWorkers) , trunc(Index * Length / NumWorkers )) || Index <- lists:seq(NumWorkers)];
		_ ->
			Workers
	end,

	Workers3 = case WorkerAgentCount =/= Length of
		true ->
			F = fun(Index) ->
				CurrWorkers = lists:nth(Index, Workers2),
				rvo2_worker:config(trunc((Index - 1) * Length / NumWorkers) , trunc(Index * Length / NumWorkers ), CurrWorkers)
			end,
			[F(Index) || Index <- lists:seq(NumWorkers)];
		false ->
			Workers2
	end,

	KdTree2 = rvo2_kd_tree:buildAgentTree(Agents, KdTree),

	F2 = fun(Worker, Simulator3) ->
		Simulator4 = rvo2_worker:step(Simulator3, Worker),
		Simulator4
	end,
	Simulator5 = lists:foldl(F2, Simulator2#rvo2_simulator{kdTree = KdTree2}, Workers3),

	F3 = fun(Worker, Simulator3) ->
		Simulator4 = rvo2_worker:update(Simulator3, Worker),
		Simulator4
	end,
	Simulator6 = lists:foldl(F3, Simulator5, Workers3),

	Simulator6#rvo2_simulator{globalTime = GlobalTime + TimeStep}.


updateDeleteAgent(Simulator = #rvo2_simulator{agents = Agents}) ->
	IsDelete = false,

	F = fun(Agent = #rvo2_agent{needDelete = false}, {IsDelete2, Acc}) ->
				{IsDelete2, [Agent | Acc]};
			(_Agent, Acc) ->
				{true, Acc}
	end,

	{IsDelete2, Agents2} = lists:foldl(F, {IsDelete, []}, Agents),

	case IsDelete2 of
		true ->
			onDelAgent(Simulator#rvo2_simulator{agents = Agents2});
		false ->
			Simulator
	end.

onDelAgent(Simulator = #rvo2_simulator{agents = Agents}) ->
	
	Len = length(Agents),

	F = fun(Index, {AgentNo2indexDict, Index2agentNoDict}) ->
			#rvo2_agent{id = AgentNo} = lists:nth(Index, Agents),
			AgentNo2indexDict2 = dict:append(AgentNo, Index, AgentNo2indexDict),
			Index2agentNoDict2 = dict:append(Index, AgentNo, Index2agentNoDict),
			{AgentNo2indexDict2, Index2agentNoDict2}
	end,
	{AgentNo2indexDict2, Index2agentNoDict2} = lists:foldl(F, {[], []}, lists:seq(1, Len)),

	Simulator#rvo2_simulator{agentNo2indexDict = AgentNo2indexDict2, index2agentNoDict = Index2agentNoDict2}.

getNumAgents(#rvo2_simulator{agents = Agents}) ->
	length(Agents).
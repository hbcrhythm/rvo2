-module(rvo2_simulator).
-author('labihbc@gmail.com').

-include("rvo2.hrl").

-define(DEFAULT_NUM_WORKER, 4).

-export([init/0, setTimeStep/2, setAgentDefaults/8, processObstacles/1, addAgent/2, addObstacle/2, doStep/1]).
-export([ 
		getNumAgents/1, 
		getAgentOrcaLines/2, 
		getAgentPosition/2, 
		getAgentPrefVelocity/2,
		getAgentRadius/2,
		getAgentTimeHorizon/2,
		getAgentTimeHorizonObst/2,
		getAgentVelocity/2,
		getNumObstacleVertices/1,
		setAgentPrefVelocity/3
	]).

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
	{KdTree2, Obstacles2} = rvo2_kd_tree:buildObstacleTree(KdTree, Simulator),
 	Simulator#rvo2_simulator{kdTree = KdTree2, obstacles = Obstacles2}.

%% @doc Adds a new obstacle to the simulation
%% @doc To add a "negative" obstacle, e.g. a bounding polygon around the environment, the vertices should be listed in clockwise order.
%% @doc Vertices 逆时针列表
addObstacle(Vertices, Simulator) when length(Vertices) < 2 ->	Simulator; 
addObstacle(Vertices, Simulator = #rvo2_simulator{obstacles = Obstacles}) ->
	LenVertices = length(Vertices),
	InitLen 	= length(Obstacles),

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

					Obstacles3 = lists:keyreplace(PreviousId, #rvo2_obstacle.id, Obstacles2, Previous2),
					{Obstacle2, Obstacles3};
				false ->
					{Obstacle, Obstacles2}
			end,

			{Obstacle5, Obstacles6} = case VerticeId == LenVertices of
				true ->
					ObstacleNoId = ?RVO2_IF(InitLen == 0, 1, InitLen + 1),

					Obstacle4 = Obstacle3#rvo2_obstacle{next_id = ObstacleNoId},

					Next = lists:keyfind(ObstacleNoId, #rvo2_obstacle.id, Obstacles4),
					
					Next2 = Next#rvo2_obstacle{previous_id = ObstacleNoId},

					Obstacles5 = lists:keyreplace(ObstacleNoId, #rvo2_obstacle.id, Obstacles4, Next2),

					{Obstacle4, Obstacles5};
				false ->
					{Obstacle3, Obstacles4}
			end,

			Obstacle6 = Obstacle5#rvo2_obstacle{direction = rvo2_match:normalize(rvo2_vector2:sub(?RVO2_IF(VerticeId == LenVertices, lists:nth(1, Vertices), lists:nth(VerticeId + 1, Vertices)), lists:nth(VerticeId, Vertices)))},

			Obstacle7 = case LenVertices of
				2 ->
					Obstacle6#rvo2_obstacle{convex = true};
				_ ->
					Convex = (rvo2_match:leftOf(lists:nth(?RVO2_IF(VerticeId == 1, LenVertices, VerticeId), Vertices), lists:nth(VerticeId, Vertices), lists:nth(?RVO2_IF(VerticeId == LenVertices, 1, VerticeId + 1), Vertices)) >= 0.0),
					Obstacle6#rvo2_obstacle{convex = Convex}
			end,
			F(T, [Obstacle7 | Obstacles6])
	end,

	Obstacles2 = F(lists:seq(1, LenVertices), Obstacles),
	Obstacles3 = lists:keysort(#rvo2_obstacle.id, Obstacles2),
	% lager:info("Obstacles3 ~w~n",[ [ {Id_, Point_} || #rvo2_obstacle{id = Id_, point = Point_} <- Obstacles3] ]),
	Simulator#rvo2_simulator{obstacles = Obstacles3}.


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

	Agents2 = lists:reverse([Agent | lists:reverse(Agents)]),


	Simulator2 = Simulator#rvo2_simulator{s_totalID = STotalId2, agents = Agents2},

	Simulator3 = onAddAgent(Simulator2),

	{STotalId, Simulator3}.

onAddAgent(Simulator = #rvo2_simulator{agents = []}) -> Simulator;
onAddAgent(Simulator = #rvo2_simulator{agents = Agents, agentNo2indexDict = AgentNo2indexDict, index2agentNoDict = Index2agentNoDict}) ->
	
	Index = length(Agents),
	
	AgentNo = (lists:nth(Index, Agents))#rvo2_agent.id,

	AgentNo2indexDict2 = dict:store(AgentNo, Index, AgentNo2indexDict),
	Index2agentNoDict2 = dict:store(Index, AgentNo, Index2agentNoDict),

	lager:info(" length(Agents ) ~w AgentNo ~w AgentNo2indexDict2 ~w Index2agentNoDict2 ~w",[length(Agents), AgentNo, dict:to_list(AgentNo2indexDict2), dict:to_list(Index2agentNoDict2)]),

	Simulator#rvo2_simulator{agentNo2indexDict = AgentNo2indexDict2, index2agentNoDict = Index2agentNoDict2}.

doStep(Simulator) ->
	Simulator2 = #rvo2_simulator{workers = Workers, numWorkers = NumWorkers, workerAgentCount = WorkerAgentCount, kdTree = KdTree, agents = Agents, globalTime = GlobalTime, timeStep = TimeStep} = updateDeleteAgent(Simulator),
	
	Length = getNumAgents(Simulator2),
	lager:info("length ~w~n",[Length]),

	Workers2 = case Workers of
		[] ->
			[rvo2_worker:init( trunc((Index - 1) * Length / NumWorkers + 1) , trunc(Index * Length / NumWorkers ) + 1) || Index <- lists:seq(1, NumWorkers)];
		_ ->
			Workers
	end,
	lager:info("Workers2 ~w~n",[Workers2]),

	Workers3 = case WorkerAgentCount =/= Length of
		true ->
			F = fun(Index) ->
				CurrWorkers = lists:nth(Index, Workers2),
				rvo2_worker:config(trunc((Index - 1) * Length / NumWorkers + 1) , trunc(Index * Length / NumWorkers + 1), CurrWorkers)
			end,
			[F(Index) || Index <- lists:seq(1, NumWorkers)];
		false ->
			Workers2
	end,

	lager:info("buildAgentTree start ~n",[]),

	KdTree2 = rvo2_kd_tree:buildAgentTree(Agents, KdTree),

	lager:info("buildAgentTree step finish ~n",[]),

	F2 = fun(Worker, Simulator3) ->
		Simulator4 = rvo2_worker:step(Simulator3, Worker),
		Simulator4
	end,
	Simulator5 = lists:foldl(F2, Simulator2#rvo2_simulator{kdTree = KdTree2}, Workers3),

	lager:info("step finish ~n",[]),

	F3 = fun(Worker, Simulator3) ->
		Simulator4 = rvo2_worker:update(Simulator3, Worker),
		% lager:info("update finished ~w",[ [{Id, Position}  || #rvo2_agent{id = Id, position = Position} <- Simulator4#rvo2_simulator.agents] ]),
		Simulator4
	end,
	Simulator6 = lists:foldl(F3, Simulator5, Workers3),

	lager:info("step update finish ~n",[]),

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
			AgentNo2indexDict2 = dict:store(AgentNo, Index, AgentNo2indexDict),
			Index2agentNoDict2 = dict:store(Index, AgentNo, Index2agentNoDict),
			{AgentNo2indexDict2, Index2agentNoDict2}
	end,
	{AgentNo2indexDict2, Index2agentNoDict2} = lists:foldl(F, {[], []}, lists:seq(1, Len)),

	Simulator#rvo2_simulator{agentNo2indexDict = AgentNo2indexDict2, index2agentNoDict = Index2agentNoDict2}.

getNumAgents(#rvo2_simulator{agents = Agents}) ->
	length(Agents).

getAgentOrcaLines(AgentNo, #rvo2_simulator{agents = Agents, agentNo2indexDict = AgentNo2indexDict}) ->
	Index = dict:fetch(AgentNo, AgentNo2indexDict),
	Agent = lists:nth(Index, Agents),
	Agent#rvo2_agent.orcaLines. 

getAgentPosition(AgentNo, #rvo2_simulator{agents = Agents, agentNo2indexDict = AgentNo2indexDict}) ->
	Index = dict:fetch(AgentNo, AgentNo2indexDict),
	Agent = lists:nth(Index, Agents),
	Agent#rvo2_agent.position.

getAgentPrefVelocity(AgentNo, #rvo2_simulator{agents = Agents, agentNo2indexDict = AgentNo2indexDict}) ->
	Index = dict:fetch(AgentNo, AgentNo2indexDict),
	Agent = lists:nth(Index, Agents),
	Agent#rvo2_agent.prefVelocity.

getAgentRadius(AgentNo, #rvo2_simulator{agents = Agents, agentNo2indexDict = AgentNo2indexDict}) ->
	Index = dict:fetch(AgentNo, AgentNo2indexDict),
	Agent = lists:nth(Index, Agents),
	Agent#rvo2_agent.radius.

getAgentTimeHorizon(AgentNo, #rvo2_simulator{agents = Agents, agentNo2indexDict = AgentNo2indexDict}) ->
	Index = dict:fetch(AgentNo, AgentNo2indexDict),
	Agent = lists:nth(Index, Agents),
	Agent#rvo2_agent.timeHorizon.

getAgentTimeHorizonObst(AgentNo, #rvo2_simulator{agents = Agents, agentNo2indexDict = AgentNo2indexDict}) ->
	Index = dict:fetch(AgentNo, AgentNo2indexDict),
	Agent = lists:nth(Index, Agents),
	Agent#rvo2_agent.timeHorizonObst.

getAgentVelocity(AgentNo, #rvo2_simulator{agents = Agents, agentNo2indexDict = AgentNo2indexDict}) ->
	Index = dict:fetch(AgentNo, AgentNo2indexDict),
	Agent = lists:nth(Index, Agents),
	Agent#rvo2_agent.velocity.

getNumObstacleVertices(#rvo2_simulator{obstacles = Obstacles}) ->
	length(Obstacles).

setAgentPrefVelocity(AgentNo, PrefVelocity, Simulator = #rvo2_simulator{agents = Agents, agentNo2indexDict = AgentNo2indexDict}) ->
	
	% lager:info("AgentNo2indexDict  ~w~n ", [dict:to_list(AgentNo2indexDict)]),

	Index = dict:fetch(AgentNo, AgentNo2indexDict),
	Agent = #rvo2_agent{id = Id} = lists:nth(Index, Agents),

	% lager:info(" setAgentPrefVelocity ~w ~n",[PrefVelocity]),
	Agent2 = Agent#rvo2_agent{prefVelocity = PrefVelocity},
	Agents2 = lists:keyreplace(Id, #rvo2_agent.id, Agents, Agent2),


	Simulator#rvo2_simulator{agents = Agents2}.
-module(rvo2_simulator).
-author('labihbc@gmail.com').

-include("rvo2.hrl").

-define(DEFAULT_NUM_WORKER, 4).

-export([setTimeStep/2, setAgentDefaults/8, processObstacles/1, addAgent/2, doStep/1]).

init() ->
	Rvo2Simulator = #rvo2_simulator{},
	Rvo2Simulator2 = setNumWorkers(0, Rvo2Simulator).

setTimeStep(TimeStep, Rvo2Simulator) ->
	Rvo2Simulator#rvo2_simulator{timeStep = TimeStep}.

setAgentDefaults(NeighborDist, MaxNeighbors, TimeHorizon, TimeHorizonObst, Radius, MaxSpeed, Velocity, Rvo2Simulator) ->
	DefaultAgent = #rvo2_agent{
		maxNeighbors = MaxNeighbors
		,maxSpeed = MaxSpeed
		,neighborDist = NeighborDist
		,radius = Radius
		,timeHorizon = TimeHorizon
		,timeHorizonObst = TimeHorizonObst
		,velocity = Velocity
	},
	Rvo2Simulator#rvo2_simulator{defaultAgent = DefaultAgent}.


setNumWorkers(NumWorkers, Rvo2Simulator) ->
	case NumWorkers =< 0 of
		true ->
			Rvo2Simulator#rvo2_simulator{numWorkers = ?DEFAULT_NUM_WORKER, workers = [], workerAgentCount = 0};
		false ->
			Rvo2Simulator#rvo2_simulator{numWorkers = NumWorkers, workers = [], workerAgentCount = 0}
	end.

processObstacles(Rvo2Simulator = #rvo2_simulator{kdTree = KdTree}) ->
	KdTree2 = rvo2_kd_tree:buildObstacleTree(KdTree, Rvo2Simulator),
	Rvo2Simulator#rvo2_simulator{kdTree = KdTree}.

addAgent(_Rvo2Vector2, Rvo2Simulator = #rvo2_simulator{defaultAgent = undefined}) ->
	{-1, Rvo2Simulator};
addAgent(Rvo2Vector2, Rvo2Simulator = #rvo2_simulator{defaultAgent = DefaultAgent, s_totalID = STotalId, agents = Agents}) ->
	Agent = #rvo2_agent{
		id 				= STotalId
		,maxNeighbors 	= DefaultAgent#maxNeighbors
		,maxSpeed 		= DefaultAgent#maxSpeed
		,neighborDist 	= DefaultAgent#neighborDist
		,position 		= Rvo2Vector2
		,radius 		= DefaultAgent#radius
		,timeHorizon 	= DefaultAgent#timeHorizon
		,timeHorizonObst= DefaultAgent#timeHorizonObst
		,velocity 		= DefaultAgent#velocity
	},
	
	STotalId2 = STotalId + 1,

	Agents2 = lists:reverse([Agent | Agents]),

	Rvo2Simulator2 = Rvo2Simulator#rvo2_simulator{s_totalID = STotalId2, agents = Agents2}.

	{STotalId2, Rvo2Simulator2}.

onAddAgent(Rvo2Simulator = #rvo2_simulator{agents = []}) -> Rvo2Simulator;
onAddAgent(Rvo2Simulator = #rvo2_simulator{agents = Agents, agentNo2indexDict = AgentNo2indexDict, index2agentNoDict = Index2agentNoDict}) ->
	Index = length(Agents),
	AgentNo = (lists:nth(Index, Agents))#rvo2_agent.id,
	AgentNo2indexDict2 = dict:append(AgentNo, Index, AgentNo2indexDict),
	Index2agentNoDict2 = dict:append(Index, AgentNo, Index2agentNoDict),

	Rvo2Simulator#rvo2_simulator{agentNo2indexDict = AgentNo2indexDict2, index2agentNoDict = Index2agentNoDict2}.

doStep(Rvo2Simulator) ->
	UpdateDeleteAgent2 = #rvo2_simulator{workers = Workers} = updateDeleteAgent(Rvo2Simulator),
	
	case Workers of
		[] ->
			
		_ ->

	end,


updateDeleteAgent(Rvo2Simulator = #rvo2_simulator{agents = Agents}) ->
	IsDelete = false,

	F = fun(Agent = #rvo2_agent{needDelete = false}, {IsDelete2, Acc}) ->
				{IsDelete2, [Agent | Acc]};
			(Agent, Acc) ->
				{true, Acc}
	end,

	{IsDelete2, Agents2} = lists:foldl(F, {IsDelete, []}, Agents),

	case IsDelete2 of
		true ->
			onDelAgent(Rvo2Simulator#{agents = Agents2}).
		false ->
			Rvo2Simulator
	end.

onDelAgent(Rvo2Simulator = #rvo2_simulator{agents = Agents}) ->
	
	Len = length(Agents),

	F = fun(Index, {AgentNo2indexDict, Index2agentNoDict}) ->
			#rvo2_agent{id = AgentNo} = lists:nth(Index, Agents),
			AgentNo2indexDict2 = dict:append(AgentNo, Index, AgentNo2indexDict),
			Index2agentNoDict2 = dict:append(Index, AgentNo, Index2agentNoDict),
			{AgentNo2indexDict2, Index2agentNoDict2}
	end,
	{AgentNo2indexDict2, Index2agentNoDict2} = lists:foldl(F, {[], []}, lists:seq(1, Len)),

	Rvo2Simulator#rvo2_simulator{agentNo2indexDict = AgentNo2indexDict2, index2agentNoDict = Index2agentNoDict2}.

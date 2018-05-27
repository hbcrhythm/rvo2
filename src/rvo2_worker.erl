-module(rvo2_worker).
-author('lbaihbc@gmail.com').

-include("rvo2.hrl").

-export([init/2, config/3, step/2, update/2]).

init(Start, End) ->
	#rvo2_worker{start_ = Start, end_ = End}.

config(Start, End, Worker) ->
	Worker#rvo2_worker{start_ = Start, end_ = End}.

step(Simulator = #rvo2_simulator{agents = Agents, timeStep = TimeStep}, #rvo2_worker{start_ = Start, end_ = End}) ->
	Agents2 = lists:sublist(Agents, Start, End - Start),

	F = fun F([Agent = #rvo2_agent{id = Id} | T], NewAgents, Simulator2 = #rvo2_simulator{agents = Agents3, obstacles = Obstacles}) ->
				
			 	Agent2 = rvo2_agent:computeNeighbors(Simulator, Agent),
				
			 	Agent3 = rvo2_agent:computeNewVelocity(TimeStep, Obstacles, Agent2),

			 	Agents4 = lists:keyreplace(Id, #rvo2_agent.id, Agents3, Agent3),

			 	F(T, [Agent3 | NewAgents], Simulator2#rvo2_simulator{agents = Agents4});
			F([], NewAgents, _Simulator2) ->
				NewAgents
	end,
	F(Agents2, [], Simulator).

update(Simulator = #rvo2_simulator{agents = Agents}, #rvo2_worker{start_ = Start, end_ = End}) ->
	Agents2 = lists:sublist(Agents, Start, End - Start),

	F = fun F([Agent = #rvo2_agent{id = Id} | T], NewAgents, Simulator2 = #rvo2_simulator{agents = Agents3}) ->
		
			 	Agent2 = rvo2_agent:update(Simulator, Agent),

			 	Agents4 = lists:keyreplace(Id, #rvo2_agent.id, Agents3, Agent2),
			 	
			 	F(T, [Agent2 | NewAgents], Simulator2#rvo2_simulator{agents = Agents4});
			F([], NewAgents, _Simulator2) ->
				NewAgents
	end,

	F(Agents2, [], Simulator).
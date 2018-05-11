-module(rvo2_worker).
-author('lbaihbc@gmail.com').

-include("rvo2.hrl").

-export([init/2, config/3, step/2, update/2]).

init(Start, End) ->
	#rvo2_worker{start = Start, end = End}.

config(Start, End, Worker) ->
	Worker#rvo2_worker{start = Start, end = End}.

step(Simulator = #rvo2_simulator{agents = Agents, timeStep = TimeStep}, #rvo2_worker{start = Start, end = End}) ->
	Agents2 = lists:sublist(Agents, 1, End),

	F = fun F([Agent = #rvo2_agent{id = Id} | T], Simulator2 = #rvo2_simulator{agents = Agents}) ->
			 	Agent2 = rvo2_agent:computeNeighbors(Simulator, Agent),
			 	Agent3 = rvo2_agent:computeNewVelocity(TimeStep, Agent2),

			 	Agents2 = lists:keyreplace(Id, #rvo2_agent.id, Agents, Agent3),
			 		

			 	F(T, Simulator2#rvo2_simulator{agents = Agents2});
			F([], Simulator2) ->
				Simulator2
	end,
	F(Agents2, Simulator).

update(Simulator, #rvo2_worker{start = Start, end = End}) ->
	Agents2 = lists:sublist(Agents, 1, End),

	F = fun F([Agent = #rvo2_agent{id = Id} | T], Simulator2 = #rvo2_simulator{agents = Agents}) ->

			 	Agent2 = rvo2_agent:update(Simulator, Agent),

			 	Agents2 = lists:keyreplace(Id, #rvo2_agent.id, Agents, Agent2),
			 		
			 	F(T, Simulator2#rvo2_simulator{agents = Agents2});
			F([], Simulator2) ->
				Simulator2
	end,

	F(Agents2, Simulator).
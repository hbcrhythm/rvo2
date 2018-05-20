-module(rvo2_worker).
-author('lbaihbc@gmail.com').

-include("rvo2.hrl").

-export([init/2, config/3, step/2, update/2]).

init(Start, End) ->
	#rvo2_worker{start_ = Start, end_ = End}.

config(Start, End, Worker) ->
	Worker#rvo2_worker{start_ = Start, end_ = End}.

step(Simulator = #rvo2_simulator{agents = Agents, timeStep = TimeStep}, #rvo2_worker{start_ = Start, end_ = End}) ->
	Agents2 = lists:sublist(Agents, Start, End - 1),

	F = fun F([Agent = #rvo2_agent{id = Id} | T], Simulator2 = #rvo2_simulator{agents = Agents3, obstacles = Obstacles}) ->
				lager:info("id ~w ================= computeNeighbors ~w ~n",[Id, Agent]),

			 	Agent2 = rvo2_agent:computeNeighbors(Simulator, Agent),

			 	lager:info("computeNewVelocity ~w~n",[Agent2]),

			 	Agent3 = rvo2_agent:computeNewVelocity(TimeStep, Obstacles, Agent2),

			 	lager:info("id ~w ================ computeNewVelocity Agent3 ~w~n",[Id, Agent3]),

			 	Agents4 = lists:keyreplace(Id, #rvo2_agent.id, Agents3, Agent3),

			 	F(T, Simulator2#rvo2_simulator{agents = Agents4});
			F([], Simulator2) ->
				Simulator2
	end,
	F(Agents2, Simulator).

update(Simulator = #rvo2_simulator{agents = Agents}, #rvo2_worker{end_ = End}) ->
	Agents2 = lists:sublist(Agents, 1, End),

	F = fun F([Agent = #rvo2_agent{id = Id} | T], Simulator2 = #rvo2_simulator{agents = Agents3}) ->
			 	Agent2 = rvo2_agent:update(Simulator, Agent),

			 	Agents4 = lists:keyreplace(Id, #rvo2_agent.id, Agents3, Agent2),
			 		
			 	F(T, Simulator2#rvo2_simulator{agents = Agents4});
			F([], Simulator2) ->
				Simulator2
	end,

	F(Agents2, Simulator).
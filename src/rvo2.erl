-module(rvo2).

%% API exports
-export([test/0]).

%%====================================================================
%% API functions
%%====================================================================

test() ->

	Rvo2Simulator = #rvo2_simulator{},
	
	Rvo2Simulator2 = rvo2_simulator:setTimeStep(0.25, Rvo2Simulator),

	Rvo2Simulator3 = rvo2_simulator:setAgentDefaults(15.0, 10, 5.0, 5.0, 2.0, 2.0, rvo2_vector:init(0.0, 0.0), Rvo2Simulator2),

	Rvo2Simulator4 = rvo2_simulator:processObstacles(Rvo2Simulator3),

	Rvo2Simulator5 = createAgent(Rvo2Simulator4),
	Rvo2Simulator6 = createAgent(Rvo2Simulator5),
	Rvo2Simulator7 = createAgent(Rvo2Simulator6),

	Rvo2Simulator7.

createAgent(Rvo2Simulator) ->
	{Sid, Rvo2Simulator2} = rvo2_simulator:addAgent(rvo2_vector2:init(3, 3)),
	case Sid >= 0 of
		true ->
			ok;
		false ->
			ok
	end,
	Rvo2Simulator2.


%%====================================================================
%% Internal functions
%%====================================================================

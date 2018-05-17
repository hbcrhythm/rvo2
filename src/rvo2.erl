-module(rvo2).

%% API exports
-export([test/0]).

-include("rvo2.hrl").

%%====================================================================
%% API functions
%%====================================================================

test() ->
	application:ensure_all_started(lager),
	Simulator = rvo2_simulator:init(),

	Simulator_1 = createObstacles(Simulator),
	Simulator_2	= createObstacles2(Simulator_1),
	Simulator_3	= createObstacles3(Simulator_2),
	Simulator_4	= createObstacles4(Simulator_3),
	
	Simulator2 = rvo2_simulator:setTimeStep(0.25, Simulator_4),

	Simulator3 = rvo2_simulator:setAgentDefaults(15.0, 10, 5.0, 5.0, 2.0, 2.0, rvo2_vector2:init(0.0, 0.0), Simulator2),

	Simulator4 = rvo2_simulator:processObstacles(Simulator3),

	Simulator5 = createAgent(0, 0, Simulator4),
	% Simulator6 = createAgent(-4.809568, 2.546146, Simulator5),
	% Simulator7 = createAgent(3.594887, -4.847359, Simulator6),
	Simulator7 = Simulator5,

	Simulator8 = rvo2_simulator:doStep(Simulator7),

	%% 循环处理
	Simulator9 = loop(4, rvo2_vector2:init(4.497988,-13.88612), [{4.387878, 2.897997E-05}, {4.015454, 4.807991E-05}, {5.733863, 6.463581E-05}, {5.352513, 1.670133E-05}] , Simulator8),

	lager:info("loop end ~p",[Simulator9]),

	Simulator9.


loop(0, _, _, Simulator) -> Simulator;
loop(Num, MousePosition, [{Angle, Dist} | TList], Simulator = #rvo2_simulator{agents = Agents}) ->
	lager:info("loop num ============================= ~w~n",[Num]),
	F = fun F([#rvo2_agent{id = Id} | T], Simulator2) ->
			
				Pos = rvo2_simulator:getAgentPosition(Id, Simulator2),
				Vel = rvo2_simulator:getAgentPrefVelocity(Id, Simulator2),

				lager:info("id ~w pos ~w vel ~w ~n",[Id, Pos, Vel]),

				lager:info("mousePosition ~w agent position ~w~n",[MousePosition, rvo2_simulator:getAgentPosition(Id, Simulator2)]),
				GoalVector = rvo2_vector2:subtract(MousePosition, rvo2_simulator:getAgentPosition(Id, Simulator2)),

				lager:info("GoalVector ~w~n", [GoalVector]),

				GoalVector3 = case rvo2_match:absSq(GoalVector) > 1.0 of
					true ->
						GoalVector2 = rvo2_match:normalize(GoalVector),
						GoalVector2;
					false ->
						GoalVector
				end,
				Simulator3 = rvo2_simulator:setAgentPrefVelocity(Id, GoalVector3, Simulator2),
				
				% Angle = rand:uniform() * 2.0 * math:pi(),
				% Dist = rand:uniform() * 0.0001,

				Simulator4 = rvo2_simulator:setAgentPrefVelocity(Id, rvo2_vector2:add(rvo2_simulator:getAgentPrefVelocity(Id, Simulator3), rvo2_vector2:multiply(Dist, rvo2_vector2:init(math:cos(Angle), math:sin(Angle)) )), Simulator3),
				F(T, Simulator4);

			F([], Simulator2) ->
				Simulator2
	end,
	Simulator2 = F(Agents, Simulator),
	loop(Num - 1, MousePosition, TList, Simulator2).

createAgent(X, Z, Simulator) ->
	{Sid, Simulator2} = rvo2_simulator:addAgent(rvo2_vector2:init(X, Z), Simulator),
	case Sid >= 0 of
		true ->
			ok;
		false ->
			ok
	end,
	Simulator3 = rvo2_simulator:setAgentPrefVelocity(Sid, rvo2_vector2:init(0, 0), Simulator2),
	Simulator3.


createObstacles(Simulator) ->
	MinX = -40,
	MinZ = 10,
	MaxX = -10,
	MaxZ = 40,
	Vertices = [rvo2_vector2:init(MaxX, MaxZ), rvo2_vector2:init(MinX, MaxZ), rvo2_vector2:init(MinX, MinZ), rvo2_vector2:init(MaxX, MinZ)],
	rvo2_simulator:addObstacle(Vertices, Simulator).

createObstacles2(Simulator) ->
	MinX = 10,
	MinZ = 10,
	MaxX = 40,
	MaxZ = 40,
	Vertices = [rvo2_vector2:init(MaxX, MaxZ), rvo2_vector2:init(MinX, MaxZ), rvo2_vector2:init(MinX, MinZ), rvo2_vector2:init(MaxX, MinZ)],
	rvo2_simulator:addObstacle(Vertices, Simulator).

createObstacles3(Simulator) ->
	MinX = 10,
	MinZ = -40,
	MaxX = 40,
	MaxZ = -10,
	Vertices = [rvo2_vector2:init(MaxX, MaxZ), rvo2_vector2:init(MinX, MaxZ), rvo2_vector2:init(MinX, MinZ), rvo2_vector2:init(MaxX, MinZ)],
	rvo2_simulator:addObstacle(Vertices, Simulator).

createObstacles4(Simulator) ->
	MinX = -40,
	MinZ = -40,
	MaxX = -10,
	MaxZ = -10,
	Vertices = [rvo2_vector2:init(MaxX, MaxZ), rvo2_vector2:init(MinX, MaxZ), rvo2_vector2:init(MinX, MinZ), rvo2_vector2:init(MaxX, MinZ)],
	rvo2_simulator:addObstacle(Vertices, Simulator).

%%====================================================================
%% Internal functions
%%====================================================================

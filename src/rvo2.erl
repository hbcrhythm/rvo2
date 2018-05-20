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
	Simulator6 = createAgent(3, 3, Simulator5),

	% Simulator6 = createAgent(-4.809568, 2.546146, Simulator5),
	% Simulator7 = createAgent(3.594887, -4.847359, Simulator6),
	
	Simulator8 = Simulator6,


	%% 循环处理
	Simulator9 = loop(4, rvo2_vector2:init(4.497988,-13.88612), {4.387878, 2.897997E-05}, Simulator8),

	lager:info("loop end ~p",[Simulator9]),

	Simulator9.


loop(0, _, _, Simulator) -> Simulator;
loop(Num, MousePosition, {Angle, Dist}, Simulator = #rvo2_simulator{agents = Agents}) ->
	
	Simulator2 = rvo2_simulator:doStep(Simulator),

	lager:info("loop num ============================= ~w~n",[Num]),
	F = fun F([#rvo2_agent{id = Id} | T], Simulator3) ->
			
				Pos = rvo2_simulator:getAgentPosition(Id, Simulator3),
				Vel = rvo2_simulator:getAgentPrefVelocity(Id, Simulator3),

				lager:info("id ~w pos ~w vel ~w ~n",[Id, Pos, Vel]),

				lager:info("mousePosition ~w agent position ~w~n",[MousePosition, rvo2_simulator:getAgentPosition(Id, Simulator3)]),
				GoalVector = rvo2_vector2:subtract(MousePosition, rvo2_simulator:getAgentPosition(Id, Simulator3)),

				lager:info("GoalVector ~w~n", [GoalVector]),

				GoalVector3 = case rvo2_match:absSq(GoalVector) > 1.0 of
					true ->
						GoalVector2 = rvo2_match:normalize(GoalVector),
						GoalVector2;
					false ->
						GoalVector
				end,
				Simulator4 = rvo2_simulator:setAgentPrefVelocity(Id, GoalVector3, Simulator3),
				
				% Angle = rand:uniform() * 2.0 * math:pi(),
				% Dist = rand:uniform() * 0.0001,

				Simulator5 = rvo2_simulator:setAgentPrefVelocity(Id, rvo2_vector2:add(rvo2_simulator:getAgentPrefVelocity(Id, Simulator4), rvo2_vector2:multiply(Dist, rvo2_vector2:init(math:cos(Angle), math:sin(Angle)) )), Simulator4),
				F(T, Simulator5);

			F([], Simulator3) ->
				Simulator3
	end,
	Simulator3 = F(Agents, Simulator2),
	loop(Num - 1, MousePosition, {Angle, Dist}, Simulator3).

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

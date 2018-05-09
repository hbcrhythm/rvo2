
-define(RVO_EPSILON, 0.00001).
-define(MAX_VALUE, 	4294967295).

-define(RVO2_IF(A, B, C), case A of true -> B; false -> C end).

-record(rvo2_vector, {
		x
		,y
	}).

-record(rvo2_obstacle, {
		next
		,previous
		,direction
		,point
		,id
		,convex
	}).

-record(rvo2_line, {
		direction
		,point
	}).

-record(rvo2_simulator, {
		end
		,start
		,timeStep
		,defaultAgent

		,s_totalID = 0
		
		,obstacles = []
		,kdTree = #rvo2_kd_tree{}

		,agents = []
		,agentNo2indexDict = dict:new()
		,index2agentNoDict = dict:new()
		,globalTime = 0.0
		,timeStep = 0.1
		,workers = []
		,numWorkers
		,workerAgentCount = 0
	}).

-record(rvo2_agent, {
		agentNeighbors = []
		,obstacleNeighbors = []
		,orcaLines = []

		,position
		,prefVelocity
		,velocity
		,id
		,maxNeighbors
		,maxSpeed
		,neighborDist
		,radius
		,timeHorizon
		,timeHorizonObst
		,needDelete = false
		,newVelocity

	}).

-record(rvo2_kd_tree, {
		obstacleTree
		,agents = []
		,agentTree = []
	}).

-record(rvo2_agent_tree_node, {
		begin
		,end
		,left
		,right
		,maxX
		,maxY
		,minX
		,minY
	}).

-record(rvo2_obstacle_tree_node, {
		obstacle
		,left
		,right
	}).

-record(rvo2_float_pair, {
		a
		,b
	}).

-record(rvo2_worker, {
		doneEvent
		,end
		,start
	}).
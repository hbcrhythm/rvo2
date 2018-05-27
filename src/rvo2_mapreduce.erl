-module(rvo2_mapreduce).
-author('labihbc@gmail.com').

-export([map_reduce/2, map_reduce/4]).

map_reduce(Map, Source) ->
    map_reduce(Map, invaild, Source, pmap).

map_reduce(Map, Reduce, Source, Type) ->
    MainS = self(),
    Ref = erlang:make_ref(),%% 唯一
    spawn(fun() ->
        S = self(),
        MapPids = lists:map(fun(I) ->
            spawn(fun() -> do_fun(S, Map, I) end)
        end, Source),
        Result = gather(MapPids, Reduce, Type),
        MainS ! {Ref, Result}
    end),
    receive 
        {Ref, Result} -> Result
    end.

gather(L,  Reduce, async) -> gather_async(L, Reduce, []);
gather(L,  Reduce,  sync) -> gather_sync(L, Reduce, []);
gather(L, _Reduce,  pmap) -> gather_pmap(L).

%% Reduce:处理中间结果。只要收到中间结果就处理,无关Source(原数据)的顺序
gather_async([],    _Reduce, Result) -> Result;
gather_async([_H|L], Reduce, Result) ->
    receive
        {_, map_reduce, MapResult} -> 
            ReduceResult = Reduce(MapResult, Result),
            gather_async(L, Reduce, ReduceResult)
    end.

%% Reduce:处理中间结果。有关Source(原数据)的顺序
gather_sync([],   _Reduce, Result) -> Result;
gather_sync([H|L], Reduce, Result) -> 
    receive
        {H, map_reduce, MapResult} -> 
            ReduceResult = Reduce(MapResult, Result),
            gather_sync(L, Reduce, ReduceResult)
    end.

%% Reduce:实现list:map的MapReduce,会忽略传值的map_reduce中的Reduce参数,因为本函数就是Reduce
gather_pmap([]) -> [];
gather_pmap([H|L]) ->
    receive
        {H, map_reduce, MapResult} -> [MapResult|gather_pmap(L)]
    end.

%% MapResult中间结果
do_fun(Parent, Map, I) -> 
    Parent ! {self(), map_reduce, (catch Map(I))}.

# Profiler creation
First off, lets create the header/implementation of or profiler. We will use the [collision check](/moveit/include/moveit_benchmark_suite/profilers/collision_check_profiler.h) profiler as an example.

## Step 1 &ndash; Create Query
We need to create a query which will contain the parameters needed to profile the `checkCollision` method. The query MUST always be derived from the `Query` base class. The `QueryID` is a unique identifier which is a map of strings. The dataset will use this information later on and can be used for filtering.


```cpp
struct CollisionCheckQuery : public Query
{
  CollisionCheckQuery(const QueryID& id,                               //
                      const RobotPtr& robot,                           //
                      const ScenePtr& scene,                           //
                      const moveit::core::RobotStatePtr& robot_state,  //
                      const collision_detection::CollisionRequest& request);

  RobotPtr robot;                                 // Wrapper around RobotModel
  ScenePtr scene;                                 // Wrapper around the PlanningScene
  moveit::core::RobotStatePtr robot_state;        //
  collision_detection::CollisionRequest request;  //
};
```
## Step 2 &ndash; Create Result
Next step is to create the Result which is usally the information return by the profiled method. Like the Query, the Result MUST be derived from the `Result` base class.
```cpp
class CollisionCheckResult : public Result
{
public:
  collision_detection::CollisionResult collision_result;
};
```
## Step 3 &ndash; Create Profiler
Afterwards, it's time to create the `Profiler` class. The profiler MUST be derived from the `ProfilerTemplate` interface which requires derived Query and Result as the template arguments. Minimally, you need to override the `runQuery` method. The `postRunQuery` is usally used to computing metrics. The `TIME` metric is always computed but the user can choose not to compute other metrics like `DISTANCE` and `CONTACTS`. So it's a good practice to create an enum that contains all the usefull metrics.

```cpp
class CollisionCheckProfiler : public ProfilerTemplate<CollisionCheckQuery, CollisionCheckResult>
{
public:
  CollisionCheckProfiler();

  // Bitmask options to select what metrics to compute for each run.
  enum Metrics
  {
    DISTANCE = 1 << 0,  //
    CONTACTS = 1 << 1,  //
  };

  void buildQueriesFromYAML(const std::string& filename) override;
  std::vector<metadata::SW> collectMetadata() override;

  CollisionCheckResult runQuery(const CollisionCheckQuery& query, Data& result) const override;
  void postRunQuery(const CollisionCheckQuery& query, CollisionCheckResult& result, Data& data) override;
};
```
### Step 4 &ndash; Create Query Builder

`TODO` Finisih this section.

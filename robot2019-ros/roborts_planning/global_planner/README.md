## global_planner代码阅读笔记

### /global_planner/global_planer_node.cpp

* local_planner <- feedback.path <- path_.pose <- current_path <-Plan()

```
Init();
GoalCallback()
{
    SetGoal(msg->goal);
    feedback.path = path_;
    as_.publishFeedback(feedback);
}
StartPlanning()
{
    PlanThread()
    {
        while()
        {
            GetRobotPose(current_start);
            current_goal = GetGoal()
            {
                return goal_;
            }
            Plan(current_start, current_goal, current_path);    //见a_star_planner.cpp
            PathVisualization(current_path)
            {
                path_.poses = current_path;
            }
            SetGoal(current_goal)
            {
                goal_ = currrent_gola;
            }
        }       
    }
}
```

### /global_planner/a_star_planner/a_star_planner.cpp

```
Plan(start, goal, path)
{
    SearchPath(start_index, goal_index, path)   //A*主算法
    {
        while (iter_index != start_index)
        {
            path.push_back(iter_pos);
        }
    }
}
```

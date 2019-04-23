%% user parameters
% world
N_obstacles = 5 ;

% planner
t_plan = 0.5 ; % if t_plan = t_move, then real time planning is enforced
t_move = 0.5 ;

% simulation
verbose_level = 10 ;

%% automated from here
A = segway_agent('max_speed',1.25,'max_yaw_rate',1) ;

P = segway_RTD_planner_static('verbose',verbose_level,'t_plan',t_plan,'t_move',t_move) ;

W = static_box_world('N_obstacles',N_obstacles,'verbose',verbose_level) ;

S = simulator(A,W,P,'allow_replan_errors',true,'verbose',verbose_level,...
              'max_sim_time',30,'max_sim_iterations',60) ;

%% run simulation
S.run ;
function solver_output = FRS_computation_example_4()
%% EXAMPLE 2: 2D Integrator System
% Compute the Forward Reachable Set (FRS) of the system w/ tracking error:
%
%   f = [k1 ; k2]
%   g = [0.2t ; -0.1k2]
%
% where z \in [-1,1]^2 and k \in [-1,1]^2. The initial conditions of z are
% the circle of radius 0.05 centered at the origin.
%
% This system has a "desired velocity" given by the trajectory parameter k,
% plus the tracking error that is dependent on time and the choice of
% parameter. If you change "k_eval" where it's called in the visualization
% function, you can choose where the little robot wants to go.
%
% This runs in about 5 s on a 3.4 GHz i7 laptop.
%
% Author: Shreyas Kousik
% Date:   12 Apr 2019

    %% solver options
    degree = 6 ; % SOS polynomial degree

    %% set up FRS variables
    t = msspoly('t', 1) ; % time t \in T
    z = msspoly('z', 2) ; % state z \in Z
    k = msspoly('k', 2) ; % parameters k \in K

    %% dynamics
    % trajectory-producing model
    f = [k(1) ; k(2)] ;
    
    % tracking error model
    g = [0.2*t ; -0.1*k(1)] ;
    
    %% define T x Z x K
    % this creates polynomials that are positive on T, Z, and K, thereby
    % defining them as semi-algebraic sets
    Z_range = [-1, 1 ; -1, 1] ; % z \in [-1,1]^2
    
    Z0_radius = 0.05 ; % z(0) \in [-0.1,0.1]

    K_range = [-1, 1 ; -1, 1] ; % k \in [-1,1]^2
    
    hZ = (z - Z_range(:,1)).*(Z_range(:,2) - z) ;
    
    hZ0 = Z0_radius - z(1)^2 - z(2)^2 ;
    
    hK = (k - K_range(:,1)).*(K_range(:,2) - k) ;

    %% create cost function
    % this outputs a function that integrates a polynomial over Z x K; we
    % use it as the cost function:
    %
    %   \int_{Z x K} w(z,k) dzdk
   
    int_ZK = boxMoments([z;k], [Z_range(:,1);K_range(:,1)], [Z_range(:,2);K_range(:,2)]);

    %% setup the problem structure
    solver_input_problem.t = t ;
    solver_input_problem.z = z ;
    solver_input_problem.k = k ;
    solver_input_problem.f = f ;
    solver_input_problem.g = g ;
    solver_input_problem.hZ = hZ ;
    solver_input_problem.hZ0 = hZ0 ;
    solver_input_problem.hK = hK ;
    solver_input_problem.cost = int_ZK ;
    solver_input_problem.degree = degree ;
    
    %% compute FRS without tracking error
    solver_output = compute_FRS(solver_input_problem) ;
    
    %% visualize FRS
    clf

    k_eval = rand(2,1) - 0.5 ; % pick a random k to evaluate
    visualize_2D_FRS(solver_output,k_eval,'b')
    title_string = ['2D FRS, k_1 = ',...
                    num2str(k_eval(1),'%0.2f'),', k_2 = ',num2str(k_eval(2),'%0.2f')] ;
    title(title_string)
end
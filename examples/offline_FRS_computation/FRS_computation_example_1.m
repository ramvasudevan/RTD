function FRS_computation_example_1()
%% EXAMPLE 1: 1D Integrator System
% Compute the Forward Reachable Set (FRS) of the scalar system:
%
%   dz/dt = -0.5k
%
% where z \in [-1,1] and k \in [-1,1]. The initial conditions for z lie in
% the interval [-0.1,0.1].
%
% This runs in about 0.5 s on a 3.4 GHz i7 laptop.
%
% Author: Shreyas Kousik
% Date:   12 Apr 2019

    %% solver options
    degree = 6 ; % SOS polynomial degree

    %% set up FRS variables
    t = msspoly('t', 1) ; % time t \in T
    z = msspoly('z', 1) ; % state z \in Z
    k = msspoly('k', 1) ; % parameters k \in K

    %% dynamics
    % trajectory-producing model
    f = -0.5*k ;
    
    %% define T x Z x K
    % this creates polynomials that are positive on T, Z, and K, thereby
    % defining them as semi-algebraic sets
    
    Z_range = [-1, 1] ; % z \in [-1,1]
    
    Z0_range = [-0.1,0.1] ; % z(0) \in [-0.1,0.1]

    K_range = [-1, 1] ; % k \in [-1,1]
    
    hZ = (z - Z_range(:,1)).*(Z_range(:,2) - z) ;
    
    hZ0 = (z - Z0_range(:,1)).*(Z0_range(:,2) - z) ;
    
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
    solver_input_problem.hZ = hZ ;
    solver_input_problem.hZ0 = hZ0 ;
    solver_input_problem.hK = hK ;
    solver_input_problem.cost = int_ZK ;
    solver_input_problem.degree = degree ;
    
    %% compute FRS
    solver_output = compute_FRS(solver_input_problem) ;
    
    %% visualize FRS
    close all
    visualize_1D_FRS(solver_output,'b',1,true)
    grid on
    title('1D FRS - thick blue contour is w >= 1, dashed is initial conditions')
end
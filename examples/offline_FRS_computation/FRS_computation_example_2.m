function FRS_computation_example_2()
%% EXAMPLE 2: 1D Nonlinear System with and without Tracking Error
% Compute the Forward Reachable Set (FRS) of the scalar system:
%
%   f = 0.7k^2 - z^2
%   g = -0.2t
%
% where z \in [-1,1] and k \in [-1,1]. The initial conditions for z lie in
% the interval [0.2,0.3].
%
% This runs in about 1.1 s on a 3.4 GHz i7 laptop. Note that it calls the
% FRS solver twice.
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
    f = 0.7*k - z^2 ;
    
    % tracking error model
    g = -0.2*t ;
    
    %% define T x Z x K
    % this creates polynomials that are positive on T, Z, and K, thereby
    % defining them as semi-algebraic sets
    
    Z_range = [-1, 1] ; % z \in [-1,1]
    
    Z0_range = [0.2,0.3] ; % z(0) \in [-0.1,0.1]

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
    
    %% compute FRS without tracking error
    solver_output_without_g = compute_FRS(solver_input_problem) ;
    
    %% compute FRS with tracking error
    solver_input_problem.g = g ;
    solver_output_with_g = compute_FRS(solver_input_problem) ;
    
    %% visualize FRS
    close all
    visualize_1D_FRS(solver_output_with_g,'r')
    visualize_1D_FRS(solver_output_without_g,'b')
    title('1D FRS - blue contour is without tracking error, red is with')
end
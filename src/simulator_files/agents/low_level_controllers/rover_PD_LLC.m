classdef rover_PD_LLC < low_level_controller
    properties
        yaw_gain = 12 ;
        yaw_rate_gain = 4 ;
        y_gain = 3 ;
        y_speed_gain = 0 ;
        x_gain = 1 ;
    end
    
    methods
        %% constructor
        function LLC = rover_PD_LLC(varargin)
            LLC@low_level_controller(varargin{:})
        end
        
        %% get control inputs
        function u = get_control_inputs(LLC,agent,t,z,varargin)
            % u = get_control_inputs(LLC,agent,t,z,T,U,Z)
            %
            % Use PD feedback to compute a desired yaw rate and speed for
            % the rover RTD agent. The output of this function is a 2-by-1
            % vector: u = [w_des ; v_des] 
            
            % extract reference trajectory time, input, and state
            T = varargin{1} ;
            U = varargin{2} ;
            Z = varargin{3} ;
            
            % get states
            x = z(1) ;
            y = z(2) ;
            psi = z(3) ;
            vy = z(5) ;
            
            % get desired state
            [z_des,u_des] = match_trajectories(t,T,Z,T,U) ;
            
            % get system parameters and gains
            k_psi = LLC.yaw_gain ;
            k_e_y = LLC.y_gain ;
            k_v_y = LLC.y_speed_gain ;
            k_e_x = LLC.x_gain ;
            l_r = agent.cm_rearwheel ;

            % get desired trajectory states
            v_ff = z_des(4) ;
            x_ff = z_des(1) ;
            y_ff = z_des(2) ;
            w_ff = z_des(6) ;
            vy_ff = l_r*w_ff ;
            psi_ff = z_des(3) ;

            % compute position error
            R = [cos(psi_ff) sin(psi_ff) ; -sin(psi_ff) cos(psi_ff)] ;
            p_err = R*[x_ff - x ; y_ff - y] ;

            % compute desired yaw rate
            w_des = k_psi*(psi_ff-psi) + k_e_y*p_err(2) + k_v_y*(vy_ff-vy)+w_ff ;

            % compute desired speed with saturation
            v_des = k_e_x*p_err(1) + v_ff ;
            if v_des < 0
                v_des = 0 ;
            end
            
            % output
            u = [w_des ; v_des] + u_des ;
        end
    end
end
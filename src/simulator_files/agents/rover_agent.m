classdef rover_agent < RTD_agent_2D
    properties
        % state indices
        speed_index = 5 ;
        lateral_speed_index = 6 ;
        yaw_rate_index = 6 ;
        steering_index = 7 ;
        
        % system parameters
        wheelbase = 0.3302 ;
        moment_of_inertia = 0.2120 ;
        mass = 7.78 ;
        cm_offset = -0.009 ; % center of mass to rear axle (negative)
        cm_rearwheel = 0.12 ; % rear wheel to center of mass (positive)
        coeff_friction = 1e6 ;
        max_wheel_angle = 0.66 ;
        
        % constants for steering rate
        steering_gain = 4.300730919846748 ;
        steering_constants = [0.224314009055080 ;
                             -0.008867066788855 ] ;
        
        % constants for acceleration/braking
        speed_gain = -0.5 ;
        acceleration_constants = [-12.5810995587748 ;
            -33.0170773577599 ;
            4.33920832891501 ;
            20.3041178298046 ;
            0.156420898500981 ;
            4.20678380627274 ;
            10.2828808092518 ;
            -0.610920415224012];
        braking_constants = [-4.11177295309464 ;
            -15.1817204116634 ;
            5.22364002070909 ] ;
        coasting_constants = [-5.55660998280113 ;
            -13.8953541919073 ;
            -2.47286920126272 ;
            0.480990612787014 ] ;
    end
    
    methods
    %% constructor
        function A = rover_agent(varargin)
            % set up default superclass values
            default_footprint = [0.6, 0.3] ;
            n_states = 7 ;
            n_inputs = 2 ;
            stopping_time = 1.5 ;
            
            % create agent 
            LLC = rover_PD_LLC() ;
            A@RTD_agent_2D('LLC',LLC,'footprint',default_footprint,...
                'n_states',n_states,'n_inputs',n_inputs,...
                'stopping_time',stopping_time,varargin{:}) ;
        end

    %% dynamics        
        function dzdt = dynamics(A,t,z,T,U,Z)
            % get states
            psi = z(3) ;
            vx = z(4) ;
            w = z(6) ;
            delta = z(7) ;
            
            % get system parameters
            m = A.mass ;
            Iz = A.moment_of_inertia ;
            l = A.wheelbase ;
            l_r = A.cm_rearwheel ;
            mo = (m*l_r^2+Iz)/l^2 ;
            
            % set up constants for steering rate
            cs1 = A.steering_constants(1) ;
            cs2 = A.steering_constants(2) ;
            kst = A.steering_gain ;
            
            % set up constants for acceleration/braking
            kp = A.speed_gain ;
            cm_coast = A.coasting_constants ;
            cm_accel = A.acceleration_constants ;
            cm_brake = A.braking_constants ;
            
            % get desired yaw rate and speed
            if isempty(Z)
                w_des = interp1(T,U(1,:),t,'previous');
                v_ff = interp1(T,U(2,:),t,'previous');
                v_des = max(0,v_ff) ;
            else
                u = A.LLC.get_control_inputs(A,t,z,T,U,Z) ;
                w_des = u(1) ;
                v_des = u(2) ;
            end
            
            % fix w_des based on v_des
            if abs(w_des) > -v_des + 3.5
                w_des = sign(w_des)*(-v_des+3.5);
            end
            
            % compute the driving force given the current and desired
            % longitudinal speeds
            if (v_des - vx) > 0.01
                u0 = -1.0 ;
            elseif (vx - v_des) > 0.01
                u0 = 3.0 ;
            else
                a0 = cm_accel(1) + cm_accel(3)*v_des + cm_accel(5)*v_des*v_des + cm_accel(8)*(w^2) ;
                a1 = cm_accel(2) + cm_accel(4)*v_des ;
                a2 = cm_accel(6) + cm_accel(7)*v_des ;
                u0_guess = (-a1-sqrt(a1*a1 - 4*a2*a0))/(2*a2) ;
                u0 = u0_guess + kp*(v_des - vx) ;
            end
            
            u0 = bound_values(u0,-1,pi) ;
            
            if u0 <= -0.35
                Frx = [1 u0 vx vx*u0 vx^2 u0^2 vx*u0^2 w^2]*cm_accel ;
            elseif u0 > 0
                Frx = [1 vx vx^2]*cm_brake;
            else
                if vx > 0.05
                    Frx = [1 u0 vx vx^2]*cm_coast;
                else
                    Frx = -5 ;
                end
            end
            
            % compute the desired steering wheel angle given desired speed
            % and yaw rate
            if v_des == 0 && vx > 0
                d_guess = atan(w_des*l/vx);
            elseif v_des > 0
                d_guess = atan(w_des*l/v_des) ;
            else
                d_guess = 0 ;
            end
            u1_guess = (d_guess - cs2)/cs1;
            u1 = bound_values(u1_guess,-3,3) ;
            
            % compute the dynamics
            delta_des = cs1*u1+cs2 ;
            delta_dot = kst*(delta_des-delta) ;
            x_dot = vx*cos(psi)-l_r*w*sin(psi) ;
            y_dot = vx*sin(psi)+l_r*w*cos(psi) ;
            psi_dot = w ;
            vx_dot = (Frx-mo*tan(delta)/cos(delta)^2*delta_dot*vx)/(m+mo*tan(delta)^2) ;  
            vy_dot = l_r*vx_dot/l*tan(delta)+l_r*vx/l*delta_dot/(cos(delta)^2) ;
            w_dot = vx_dot/l*tan(delta)+vx/l*delta_dot/(cos(delta)^2) ;
            
            % create output
            dzdt = [x_dot ;
                    y_dot ;
                    psi_dot ;
                    vx_dot
                    vy_dot ;
                    w_dot ;
                    delta_dot] ;
        end
    end
end

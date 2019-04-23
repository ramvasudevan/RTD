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
        
        % control gains
        yaw_gain = 12 ;
        yaw_rate_gain = 4 ;
        y_gain = 3 ;
        y_speed_gain = 0 ;
        x_gain = 1 ;
        steering_gain = 4.300730919846748 ;
        
        % constants for steering rate
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
            A@RTD_agent_2D(varargin{:},'footprint',default_footprint,...
                'n_states',n_states,'n_inputs',n_inputs,...
                'stopping_time',stopping_time) ;
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
            lr = A.cm_rearwheel ;
            mo = (m*lr^2+Iz)/l^2 ;
            
            % set up constants for steering rate
            cs1 = A.steering_constants(1) ;
            cs2 = A.steering_constants(2) ;
            kst = A.steering_gain ;
            kw = A.yaw_rate_gain ;
            
            % set up constants for acceleration/braking
            kp = A.speed_gain ;
            cm_coast = A.coasting_constants ;
            cm_accel = A.acceleration_constants ;
            cm_brake = A.braking_constants ;
            
            % get desired yaw rate and speed
            if isempty(Z)
                w_des = interp1(T,U(1,:),t,'previous');
                v_ff = interp1(T,U(2,:),t,'previous');
            else
                z_des = interp1(T',Z',t,'previous')';
                [w_ff,v_ff] = A.get_desired_yaw_rate_and_speed(z,z_des);
                w_des = w_ff+kw*(w_ff-w);                
            end
            
            v_des = max(0,v_ff) ;
            
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
            delta_des = cs1*u1+cs2 ;
            delta_dot = kst*(delta_des-delta) ;
            
            % compute the dynamics
            x_dot = vx*cos(psi)-lr*w*sin(psi) ;
            y_dot = vx*sin(psi)+lr*w*cos(psi) ;
            psi_dot = w ;
            vx_dot = (Frx-mo*tan(delta)/cos(delta)^2*delta_dot*vx)/(m+mo*tan(delta)^2) ;  
            vy_dot = lr*vx_dot/l*tan(delta)+lr*vx/l*delta_dot/(cos(delta)^2) ;
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
        
        
        function [w_des,v_des] = get_desired_yaw_rate_and_speed(A,z,z_des)
            % get states
            x = z(1) ;
            y = z(2) ;
            psi = z(3) ;
            vy = z(5) ;
            
            % get system parameters and gains
            kpsi = A.yaw_gain ;
            key = A.y_gain ;
            kvy = A.y_speed_gain ;
            kex = A.x_gain ;
            lr = A.cm_rearwheel ;

            % get desired trajectory states
            v_ff = z_des(4) ;
            x_ff = z_des(1) ;
            y_ff = z_des(2) ;
            w_ff = z_des(6) ;
            vy_ff = lr*w_ff ;
            psi_ff = z_des(3) ;

            % compute position error
            R = [cos(psi_ff) sin(psi_ff) ; -sin(psi_ff) cos(psi_ff)] ;
            err = R*[x_ff - x ; y_ff - y] ;

            % compute desired yaw rate
            w_des = kpsi*(psi_ff-psi)+key*err(2)+kvy*(vy_ff-vy)+w_ff ;

            % compute desired speed
            v_des = kex*err(1) + v_ff ;
            if v_ff == 0 || v_des < 0
                v_des = 0;
            end
        end
    end
end

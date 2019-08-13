classdef segway_agent < RTD_agent_2D
    properties
        % system parameters
        max_speed = 2 ; % m/s
        max_accel = 3.75 % m/s^2
        max_yaw_rate = 1.25 ; % rad/s
        max_yaw_accel = 5.9 ; % rad/s^2 ;
        
        % state indices
        yaw_rate_index = 4 ;
        speed_index = 5 ;
        
        % gains
        accel_motor_gain = 3.0 ;
        yaw_accel_motor_gain = 2.95 ;
        heading_gain = 10 ;
        speed_gain = 10 ;
    end
    
    methods
        %% constructor
        function A = segway_agent(varargin)
            % set up default superclass values
            default_footprint = 0.38 ;
            n_states = 5 ;
            n_inputs = 2 ;
            stopping_time = 1.5 ;
            sensor_radius = 4 ;
            
            % create agent
            A@RTD_agent_2D('footprint',default_footprint,...
                'n_states',n_states,'n_inputs',n_inputs,...
                'stopping_time',stopping_time,'sensor_radius',sensor_radius,...
                varargin{:}) ;
        end
        
        %% dynamics
        function zd = dynamics(A,t,z,T,U,Z)
            if nargin < 6
                Z = [] ;
            end
            
            % ZOH for inputs
            wdes = interp1(T,U(1,:),t,'previous') ; % yaw rate
            vdes = interp1(T,U(2,:),t,'previous') ; % speed
            
            % extract the states
            h = z(A.heading_index) ;
            w = z(A.yaw_rate_index) ;
            v = z(A.speed_index) ;
            
            % determine the inputs
            Kg = A.yaw_accel_motor_gain ;
            Ka = A.accel_motor_gain ;
            
            % if a trajectory to follow was provided, try to follow it
            if ~isempty(Z)
                htraj = interp1(T,Z(3,:),t) ;
                vtraj = interp1(T,Z(5,:),t) ;
                
                Kh = A.heading_gain ;
                Kv = A.speed_gain ;
                
                wdes = wdes + Kh*(htraj - h) ;
                vdes = vdes + Kv*(vtraj - v) ;
            end
            
            if vdes > A.max_speed
                vdes = A.max_speed;
            elseif vdes < 0
                vdes = 0;
            end
            
            if wdes > A.max_yaw_rate
                wdes = A.max_yaw_rate;
            elseif wdes < -A.max_yaw_rate
                wdes = -A.max_yaw_rate;
            end
            
            g = Kg*(wdes - w) ;
            a = Ka*(vdes - v) ;
            
            % saturate the inputs
            g = max(min(g,A.max_yaw_accel),-A.max_yaw_accel) ;
            a = max(min(a,A.max_accel),-A.max_accel) ;
            
            % calculate the derivatives
            xd = v*cos(h) ;
            yd = v*sin(h) ;
            hd = w ;
            wd = g ;
            vd = a ;
            
            % return state derivative
            zd = [xd ; yd ; hd ; wd ; vd] ;
        end
    end
end
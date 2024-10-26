classdef ResolvedRateMotionControldoesntwork
    properties
        robot  
        dt     
    end
    
    methods
        function obj = ResolvedRateControl(robot, dt)
            obj.robot = robot;  
            obj.dt = dt;        
        end
        
        function q = control(obj, q, v_e)
            J = obj.robot.jacob0(q);
            q_dot = pinv(J) * v_e;
            q = q + q_dot' * obj.dt;
        end
        
        function simulate(obj, q_initial, v_e, duration)
            q = q_initial;  
            for t = 0:obj.dt:duration
                q = obj.control(q, v_e);
                obj.robot.plot(q);
                pause(obj.dt);
            end
        end
    end
end

classdef VariableTarget < handle
    
    %% ============================
    %% ========  public  ==========
    %% ============================
    
    methods (Access = public)
        
        %% Constructor
        % param[in] yg0: initial target
        % param[in] yg_new: new target
        % param[in] t_change: time instant to start changing the target
        % param[in] a_g: target's rate of change (default = 10). Set to Inf
        %                for instantaneous (discontinuous change).
        % param[in] Ts: update target to new value every Ts sec (default = 0.0333)
        function this = VariableTarget(yg0, yg_new, t_change, a_g, Ts)
            
            if (nargin < 4), a_g = 10; end
            if (nargin < 5), Ts = 0.0333; end % update at 30 Hz
            
            this.yg0 = yg0;
            this.yg = yg0;
            this.yg_new = yg_new;
            this.a_g = a_g;
            this.Ts = Ts;
            this.t_change = t_change;
            
            this.t_update = t_change;

        end
        
        %% Returns the target at the current time instant.
        % param[in] t: time instant
        % return: the value of the target
        function yg = getTarget(this, t)
            
            if (t - this.t_update < this.Ts)
                yg = this.yg;
            else
                this.integrateTarget(this.t_update, t);
                this.t_update = t;
                yg = this.yg;
            end
            
        end
        
        %% Reinitializes the object
        function reset(this)
            
            this.t_update = this.t_change;
            this.yg = this.yg0;
            
        end
        
    end
    
    %% ===============================
    %% ========  protected  ==========
    %% ===============================
    
    methods (Access = protected)
        
        function integrateTarget(this, t0, tf)
        
            if (isinf(this.a_g))
                this.yg = this.yg_new;
                return;
            end
                
            % ode
            ode_fun = @(t, state) this.a_g*(this.yg_new - state);
            [~,state_data] = ode45(@(t,state)ode_fun(t,state), [t0 tf], this.yg);
            this.yg = state_data(end,:)';
            
%             % euler
%             yg_dot = this.a_g*(this.yg_new - this.yg);
%             this.yg = this.yg + yg_dot*(tf - t0);
            
        end
    end
    
    properties (Access = protected)

        t_change % time instant to start changing the target
        
        t_update % last time instant that the target was updated
        
        Ts % update target to new value every Ts
        
        yg0 % initial value of the target
        yg % current value of the target
        
        yg_new % new target value
        
        a_g % target update rate from yg0 to yg_new
        
    end
    
end
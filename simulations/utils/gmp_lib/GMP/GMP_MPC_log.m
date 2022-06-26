
classdef GMP_MPC_log < handle
    
    methods (Access = public)
        
        function this = GMP_MPC_log()
            
            this.clear();
            
        end
        
        function clear(this)
            
            this.n_e_data = {};
            this.p_e_data = {};
            this.p_data = {};
            this.c_data = {};
            this.si_data = [];
            this.yd_points = [];
            this.y_pred_points = [];
            this.y_current = [];
            
        end
        
    end


    properties (Access = public)
       
        n_e_data
        p_e_data
        p_data
        si_data
        yd_points
        y_pred_points
        y_current
        c_data
        
    end
    
    
end
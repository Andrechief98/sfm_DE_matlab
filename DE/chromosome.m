classdef chromosome < handle
    properties
        A;
        B;
        radius;
        alfa;
        Vd;
        lambda;

        name;
    end

    methods 
        function obj = chromosome(parameter_constraints)
            if nargin > 0
                obj.A = parameter_constraints(1,1) + rand*(parameter_constraints(1,2)-parameter_constraints(1,1));
                obj.B = parameter_constraints(2,1) + rand*(parameter_constraints(2,2)-parameter_constraints(2,1));
                obj.radius = parameter_constraints(3,1) + rand*(parameter_constraints(3,2)-parameter_constraints(3,1));
                obj.alfa = parameter_constraints(4,1) + rand*(parameter_constraints(4,2)-parameter_constraints(4,1));
                obj.Vd = parameter_constraints(5,1) + rand*(parameter_constraints(5,2)-parameter_constraints(5,1));
                obj.lambda = parameter_constraints(6,1) + rand*(parameter_constraints(6,2)-parameter_constraints(6,1));
            end
        end

        function set_parameters(obj,parameters)
            obj.A = parameters(1);
            obj.B = parameters(2);
            obj.radius = parameters(3);
            obj.alfa = parameters(4);
            obj.Vd = parameters(5);
            obj.lambda = parameters(6);
        end

        function update_parameters(obj,target_vector,donor_vector,Cr,D)
            parameters_obj=[obj.A obj.B obj.radius obj.alfa obj.Vd obj.lambda];
            parameters_target=[target_vector.A target_vector.B target_vector.radius target_vector.alfa target_vector.Vd target_vector.lambda];
            parameters_donor=[donor_vector.A donor_vector.B donor_vector.radius donor_vector.alfa donor_vector.Vd donor_vector.lambda];

            j_index=randi([1,D],1,1);

            for j=1:length(parameters_obj)
                if(rand<=Cr || j==j_index)
                    parameters_obj(j)=parameters_donor(j);
                else
                    parameters_obj(j)=parameters_target(j);
                end
            end
            
            obj.set_parameters(parameters_obj);
            
            
        end

    end

end
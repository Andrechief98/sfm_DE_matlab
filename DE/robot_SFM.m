classdef robot_SFM < handle
    properties
        e=[0 0];        %desired direction
        n_ped=[0 0];
        n_obs=[];
        n_wall=[];

        cos_gamma_obs;
        cos_gamma_ped;
        cos_gamma_wall;
        
        A;
        B;
        r;
        alfa;
        V_desired;
        lambda;

        pedestrian_list;
        acc_max=1;

        curr_pos=[];    %current position of the robot
        goal_pos=[];    %current goal of the robot
        start_info=[];
        goal_info=[];

        trajectory=[];

        obs_coor=[];    %coordinates of the single obstacle in the map
        near_wall_coor=[];  %coordinates of the nearest point of the wall to the robot
        other_ped_pos=[];   %coordinates matrix of other pedestrian positions

        V_curr=[0 0];      %current speed of the robot
        F_goal=[];      %attractive force
        F_rep_obs=[];   %repulsive force (obstacle)
        F_rep_wall=[];  %repulsive force (wall)

        %repulsive force (pedestrian)
        F_rep_ped_matrix=[]
        F_rep_ped_tot=[];   

        F_tot=[]        %total force 

        color="k";
    end

    methods

        function obj = robot_SFM(ped_list)
         if nargin > 0
            obj.pedestrian_list=ped_list;
            
         end
        end

        function set_properties(obj, chromosome)
            obj.A = chromosome.A;
            obj.B = chromosome.B;
            obj.r = chromosome.radius;
            obj.alfa = chromosome.alfa;
            obj.V_desired = chromosome.Vd;
            obj.lambda = chromosome.lambda;
        end

        function get_start_info(obj,ped_to_sub)
            for j=2:length(obj.pedestrian_list(ped_to_sub).trajectory(:,1))
                if (obj.pedestrian_list(ped_to_sub).trajectory(j,2) ~= (-99999999/1000) && obj.pedestrian_list(ped_to_sub).trajectory(j-1,2) == (-99999999/1000))
                    obj.start_info=[obj.start_info; obj.pedestrian_list(ped_to_sub).trajectory(j,:)];      
                else
                    continue
                end
            end

        end

        function get_goal_info(obj,ped_to_sub)
            for j=2:length(obj.pedestrian_list(ped_to_sub).trajectory(:,1))
                if (obj.pedestrian_list(ped_to_sub).trajectory(j,2) == (-99999999/1000) && obj.pedestrian_list(ped_to_sub).trajectory(j-1,2) ~= (-99999999/1000))
                     obj.goal_info=[obj.goal_info; obj.pedestrian_list(ped_to_sub).trajectory(j-1,:)];  
                else
                    continue
                end
            end
        end

        function clear_info(obj)
            norms=zeros(length(obj.start_info(:,1)),1);
            for j=1:length(obj.start_info(:,1)) 
                norms(j)= norm(obj.goal_info(j,2:3)-obj.start_info(j,2:3));
            end
            index=find(norms<=1);
            obj.start_info(index,:)=[];
            obj.goal_info(index,:)=[];
        end

        function set_start_pos(obj,info_index)
            obj.curr_pos=obj.start_info(info_index,2:3);
            obj.trajectory=[obj.trajectory; obj.start_info(info_index,:)];
        end

        function set_goal(obj,info_index)
            obj.goal_pos=obj.goal_info(info_index,2:3);
        end
        
        function set_obs_positions(obj, static_obs)
            obs_x_coord=(sum(static_obs(:,1)))/(length(static_obs(:,1)));
            obs_y_coord=(sum(static_obs(:,2)))/(length(static_obs(:,2)));

            obj.obs_coor=[obs_x_coord obs_y_coord];
        end

        function set_near_wall(obj,wall)
            min_norm=1000;
            obj.near_wall_coor=wall(wall==min(norm(wall)))
            for i=1:length(wall(:,1))
                if(norm(wall(i,:))<=min_norm)
                    min_norm=norm(wall(i,:));
                    obj.near_wall_coor=wall(i,:);
                else
                    continue
                end
            end
        end

        function set_other_Ped_position(obj,lista_ped,curr_time)
            index=int32(curr_time*100);
            for i=1:length(lista_ped)
                obj.other_ped_pos(i,:)=lista_ped(i).trajectory(index,2:3);
            end
        end

        function compute_desired_direction(obj)
            obj.e=(obj.goal_pos-obj.curr_pos)./norm(obj.goal_pos-obj.curr_pos);
        end
        
        function compute_Attractive_force(obj)
            obj.compute_desired_direction()
            obj.F_goal=(obj.V_desired*obj.e - obj.V_curr)./obj.alfa;
        end

        function compute_Rep_Obs_force(obj)
            obj.n_obs=(obj.curr_pos-obj.obs_coor)./norm(obj.curr_pos-obj.obs_coor);
            obj.cos_gamma_obs=(dot(obj.e,-obj.n_obs))/(norm(obj.e)*norm(-obj.n_obs));
            F_fov_obs=obj.lambda+(1-obj.lambda)*((1+obj.cos_gamma_obs)/2);
            
            obj.F_rep_obs=exp((1.4 - norm(obj.curr_pos- ...
                    obj.obs_coor))/obj.r)*F_fov_obs*obj.n_obs;
        end

        function compute_Rep_Ped_force(obj,ped_to_sub)
            obj.F_rep_ped_matrix=zeros(length(obj.other_ped_pos(:,1)),2);
            
            for i=1:length(obj.other_ped_pos(:,1))
                obj.n_ped=(obj.curr_pos-obj.other_ped_pos(i,:))./norm(obj.curr_pos-obj.other_ped_pos(i,:));
                obj.cos_gamma_ped=(dot(obj.e,-obj.n_ped))/(norm(obj.e)*norm(-obj.n_ped));
                F_fov_ped=obj.lambda+(1-obj.lambda)*((1+obj.cos_gamma_ped)/2);

                obj.F_rep_ped_matrix(i,:)=obj.A*exp((obj.r - norm(obj.curr_pos- ...
                    obj.other_ped_pos(i,:)))/obj.B)*F_fov_ped*obj.n_ped;
            end

            %bisogna sommare le forze che tutti i pedoni esercitano sul
            %robot (tranne quella del pedone che il robot sta sostituendo)
            column_x=[obj.F_rep_ped_matrix(1:ped_to_sub-1,1); obj.F_rep_ped_matrix(ped_to_sub+1:end,1)];
            column_y=[obj.F_rep_ped_matrix(1:ped_to_sub-1,2); obj.F_rep_ped_matrix(ped_to_sub+1:end,2)];
            res_x=sum(column_x);
            res_y=sum(column_y);
            obj.F_rep_ped_tot=[res_x res_y];
        end

        function compute_Rep_wall_force(obj)
            obj.n_wall=(obj.curr_pos-obj.near_wall_coor)./norm(obj.curr_pos-obj.near_wall_coor);
            obj.cos_gamma_wall=(dot(obj.e,-obj.n_wall))/(norm(obj.e)*norm(-obj.n_wall));
            F_fov_wall=obj.lambda+(1-obj.lambda)*((1+obj.cos_gamma_wall)/2);
            
            obj.F_rep_wall=exp((1.4 - norm(obj.curr_pos- ...
                    obj.near_wall_coor))/obj.r)*F_fov_wall*obj.n_wall;
        end

        function compute_Total_force(obj,ped_to_sub)
            obj.compute_Attractive_force();
            obj.compute_Rep_Obs_force();
            obj.compute_Rep_Ped_force(ped_to_sub);
            %obj.compute_Rep_wall_force();
            obj.F_tot=obj.F_goal+obj.F_rep_ped_tot+obj.F_rep_obs;%+obj.F_rep_wall;
        end

        function move_one_step(obj,ped_to_sub,curr_time,time_step)
            
            obj.set_other_Ped_position(obj.pedestrian_list,curr_time);
            
            obj.compute_Total_force(ped_to_sub);

            %controllo limite accelerazione
            if(norm(obj.F_tot*time_step)>obj.acc_max)
                obj.F_tot=[sign(obj.F_tot(1))*obj.acc_max/time_step sign(obj.F_tot(2))*obj.acc_max/time_step];
            end
            obj.V_curr = obj.V_curr + time_step*obj.F_tot;
            
            %controllo limite velocità
            if(norm(obj.V_curr)>obj.V_desired)
                obj.V_curr=[sign(obj.V_curr(1))*obj.V_desired sign(obj.V_curr(2))*obj.V_desired];
            end

            obj.curr_pos = obj.curr_pos + obj.V_curr*time_step;
            
        end

    end
end

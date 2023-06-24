classdef pedestrian < handle
    properties
        ID;
        trajectory=[];
        color=[];
        radius;
    end

    methods

        function obj = pedestrian(id,traj,color,radius)
         if nargin > 0
            obj.ID = id;
            traj(isnan(traj))=-99999999;
            obj.trajectory=traj;
            obj.trajectory(:,2:3)=obj.trajectory(:,2:3)./1000;
            obj.color=color;
            obj.radius=radius;
            
         end
        end


        function update_goal(obj, curr_time)
            index=int32(curr_time*100);
            if (obj.trajectory(index,2) ~= (-99999999/1000))
                for i=1:(length(obj.trajectory(index:end,2))-1)
                    if (obj.trajectory(index+i,2)== (-99999999/1000))
                        obj.goal=obj.trajectory(index+i-1,2:3);
                        break
                    else
                        continue
                    end
                end
            else
                obj.goal=[];
            end
        end
        
    end

end
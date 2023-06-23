function [obs_coordinates, wall_coordinates] = extract_obs(map_x_coordinates,map_y_coordinates)
obs_coordinates=-99999*ones(length(map_x_coordinates),2);
wall_coordinates=-99999*ones(length(map_x_coordinates),2);
obs_counter=1;
wall_counter=1;

for i=1:length(map_x_coordinates)
    temp_coordinate_vector=[map_x_coordinates(i) map_y_coordinates(i)];
    if (norm(temp_coordinate_vector)<=1.5)
        obs_coordinates(obs_counter,:)= temp_coordinate_vector;
        obs_counter=obs_counter+1;
    else
        wall_coordinates(wall_counter,:)= temp_coordinate_vector;
        wall_counter=wall_counter+1;
    end
end
obs_coordinates(obs_counter:end,:)=[];
wall_coordinates(wall_counter:end,:)=[];
end


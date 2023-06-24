clear 
close
clc

ESTRAZIONE_DATI_EXP1_RUN1
loadMap

[static_obs, wall]=extract_obs(obst_x,obst_y);


ped_list=[];
temp_traj=[];

%vettore di colori
body_colors = [
    0,0,0
    0,0,1
    0,1,0
    1.0,0.2,0.2
    1,1,0
    1,0,1
    0,1,1
    1,0.8,0
    0,0.6,1
    0.8,1,0
    0.5,0.5,0.5
    0.8,0.8,0.8
    0.1,0.1,0.1];

% CREAZIONE LISTA PEDONI
for i=1:13
    temp_traj=[data_1_1.Time(:) data_1_1.X(:,i) data_1_1.Y(:,i)];
    ped_list=[ped_list pedestrian(i,temp_traj,body_colors(i,:),0.6)];
end

% CREAZIONE PARAMETRI SOCIAL FORCE MODEL
A=1.2;
B=0.8;
radius=1;
alfa=0.3;
Vd=1.2;
lambda=0.5;

SFM_parameters=[A B radius alfa Vd lambda];

% CREAZIONE ROBOT
robot=robot_SFM(SFM_parameters, ped_list);
robot.set_obs_positions(static_obs, wall);

% %prova con t=4.46 e pedone da sostituire =3
% ped_list(3).update_goal(4.46)
% ped_list(3).set_start_pos(4.46)
% 
% robot.set_goal_position(ped_list, 3)
% robot.set_start_position(ped_list, 3, 4.46)
% 
% robot.set_obs_positions(static_obs, wall);
% robot.set_other_Ped_position(ped_list,446);
% 
% robot.move_one_step(3);
% robot.set_start_position(ped_list, 3, 4.47)
%% SIMULATION

%INFO GOAL
Goal_1=[4.440, 8.550];
Goal_2=[9.030, 3.580];
Goal_3=[1.110, -3.447];
Goal_4=[-5.468, -6.159];
Goal_5=[-0.130, 4.150];

Goals=[Goal_1; Goal_2; Goal_3; Goal_4; Goal_5];
goal_string="Goal";

%PEDONE DA SOSTITUIRE
ped_to_skip=5;
robot.get_goal_info(ped_to_skip)
robot.get_start_info(ped_to_skip)
robot.cler_info()

frame_skip=20;
time_step=frame_skip/100;
simulation_time=[robot.start_info(:,1) robot.goal_info(:,1)];
%%

figure()
hold on
axis equal
axis([-8,10,-8,10])
legend('Obstacle','','','','','','Helmet 1','Helmet 2','Helmet 3','Helmet 4','Helmet 5','Helmet 6','Helmet 7','Helmet 8','Helmet 9','Helmet 10','Velodyne','Citi','Location','northwest')
hold on

for k=1:length(simulation_time)
    k;
    
    % CHIAMATA DIFFERENTIAL ALGORITHM PER OTTENERE I NUOVI PARAMETRI DEL
    % MODELLO PRIMA DI OGNI NUOVA START-GOAL TRAJECTORY

    for t=simulation_time(k,1):time_step:simulation_time(k,2)
        index=int32(t*100);
        robot.goal_pos
        comparazione=[robot.curr_pos; ped_list(ped_to_skip).trajectory(index,2:3)]

        if t==simulation_time(k,1)
            %è il primo timestep del ciclo per cui setto goal e start
            %position del robot
            info_index=find(robot.start_info(:,1)==t);
            robot.set_start_pos(info_index);
            robot.set_goal(info_index);
        end

        %plotto la mappa
        plotMap(obst_x,obst_y)
        rectangle('Position',[1.8 -5.6 7.8 1.2],'FaceColor',[1 1 1])
        text(2, -5, strcat('Frame-', int2str(index)),'FontWeight','bold','FontSize',16,'Color',[0,0,0])
    
        %plotto i goal possibili
        for k=1:length(Goals)
            plot(Goals(k,1),Goals(k,2)+0.1,".","MarkerSize",30,"Color","r");
            text(Goals(k,1),Goals(k,2)+0.1,strcat(goal_string,int2str(k)),"Color", "r")
        end
    
        %PLOTTING PEDONI
        for i=2:length(ped_list)
            %a ogni nuovo ciclo plotto la posizione del pedoni
%             if(i~=ped_to_skip)
                pos=ped_list(i).trajectory(index,2:3);
                color=ped_list(i).color(:);
                %plotto solo i pedoni da considerare
                if pos~=[-99999999 -99999999]
                    plot(pos(1),pos(2),'.','MarkerSize',20,'Color',color)
                    text(pos(1),pos(2),int2str(i))
        %             if(i<12)
        %                 text(pos(1),pos(2),int2str(i-1))
        %             end
                    hold on
                else
                    continue
                end
%             else
                pos=robot.curr_pos;
                color=robot.color;
                plot(pos(1),pos(2),'.','MarkerSize',20,'Color',color)
                text(pos(1),pos(2),"robot")
%             end
    %         
        end
    
        legend('Obstacle','','','','','','Helmet 1','Helmet 2','Helmet 3','Helmet 4','Helmet 5','Helmet 6','Helmet 7','Helmet 8','Helmet 9','Helmet 10','Velodyne','Citi','Location','northwest')
        pause(0.000001)
        clf
        hold on
        axis equal
        axis([-8,10,-8,10])

        robot.move_one_step(ped_to_skip,t,time_step)
    
    end    
end
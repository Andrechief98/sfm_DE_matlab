clear 
close
clc

ESTRAZIONE_DATI_EXP1_RUN1
loadMap

[static_obs, wall]=extract_obs(obst_x,obst_y);

% wall_to_consider=[];
% counter=1;
% for i=1:length(wall)
%     if (norm(wall(i,:))<=15)
%         wall_to_consider(counter,:)=wall(i,:);
%         counter=counter+1;
%     end
% end
% 
% clearvars -except wall_to_consider 
% filename="wallForce.mat"
% save(filename)
% load("wallForce.mat")

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
    ped_list=[ped_list pedestrian(i,temp_traj,body_colors(i,:),0.5)];
end

% PARAMETRI DIFFERENTIAL ALGORITHM
D=6;    %numero parametri da stimare (A,B,radius,alfa,Vd,lambda)
Np=5*D; %numero cromosomi nella popolazione
F=0.5;  %Mutation factor
Cr=0.8; %Crossover Rate
Ng=10;


% GENERAZIONE 0 DEI PARAMETRI

A_constraints=[0.3, 2];
B_constraints=[0.3, 1.5];
radius_constraints=[0.15, 1.5];
alfa_constraints=[0.1, 1];
Vd_constraints=[0.5, 2];
lambda_constraints=[0.1, 1];

parameters_constraints=[A_constraints;
                        B_constraints; 
                        radius_constraints; 
                        alfa_constraints; 
                        Vd_constraints; 
                        lambda_constraints];

population=[];
for i=1:Np
    population=[population chromosome(parameters_constraints)];
end

% CREAZIONE ROBOT
robot=robot_SFM(ped_list);
robot.set_obs_positions(static_obs);


%% DIFFERENTIAL ALGORITHM

frame_skip=20;
time_step=frame_skip/100;


%Creiamo un oggett che ci serve per salvare il best chromosome di ogni
%generazione
best_fitness_value=1000;
best_chromosome=chromosome([0 0; 0 0; 0 0; 0 0; 0 0; 0 0]);

%la useremo per valutare il fitness value minore a ogni generazione
plot_fitness_values_matrix=zeros(Ng,2);

%per ogni cromosoma della population è necessario andare a eseguire tutta
%la simulazione (sostituendo probabilmente tutti i pedoni... per ora
%facciamo con 1 solo pedone, il 5)distance_robot_pedestrian(2,:)

for n_gen=1:Ng
    %ad ogni nuova generazione devo valutare tutti i cromosomi della
    %popolazione
    n_gen
    new_population=[];
    best_fitness_value_of_curr_generation=1000;

%     if n_gen==2
%         break
%     end

    for n_chrom=1:Np
        n_chrom;
        %la simulazione in cui il robot viene sostituito deve essere
        %eseguita sia per il target vector X_i che per il trial vector U_i

        target_vector_X=population(n_chrom);
        random_indexes=randi([1,Np],1,3);
        while (find(random_indexes==n_chrom)~=0)
            random_indexes=randi([1,Np],1,3);
        end
        
        % **** MUTATION ****
        random_vector_Xr1= population(random_indexes(1));
        random_vector_Xr2= population(random_indexes(2));
        random_vecotr_Xr3= population(random_indexes(3));
        donor_vector_parameters = generate_donor_vector_parameters(random_vector_Xr1,random_vector_Xr2,random_vecotr_Xr3,F,parameters_constraints);
        donor_vector_V = chromosome([0 0; 0 0; 0 0; 0 0; 0 0; 0 0]);
        donor_vector_V.set_parameters(donor_vector_parameters);

        % **** BINOMIAL CROSSOVER ****
        trial_vector_U = chromosome([0 0; 0 0; 0 0; 0 0; 0 0; 0 0]);
        trial_vector_U.update_parameters(target_vector_X,donor_vector_V,Cr,D);
        
        % **** SELECTION ****
        %la selezione tra trial vector U e target vector X è fatta andando
        %a considerare quale cromosoma determina una simulazione migliore
        possible_chromosome=[target_vector_X trial_vector_U];
        
        fitness_values=zeros(1,2); %conterrà i valori della fitness functon del target vector e trial vector
        
        for chromosome_index=1:length(possible_chromosome)
            %imposto come parametri del SFM del robot il cromosoma
            %considerato.

            %per ogni nuovo cromosoma la simulazione deve ripartire da zero
            robot.set_properties(possible_chromosome(chromosome_index));
            robot.goal_info=[];
            robot.start_info=[];

            for ped_to_skip=5:5
                %ricavo le informazioni di partenza e goal del pedone da
                %sostituire
                robot.get_goal_info(ped_to_skip)
                robot.get_start_info(ped_to_skip)
                robot.clear_info()

                
                if (isempty(robot.start_info) || isempty(robot.goal_info) || (length(robot.start_info)~= length(robot.goal_info)))
                    continue
                end

                simulation_time=[robot.start_info(:,1) robot.goal_info(:,1)];
                
                %faccio partire la simulazione
                distance_robot_pedestrian=[];
                for k=1:length(simulation_time)
                    for t=simulation_time(k,1):time_step:simulation_time(k,2)
                        index=int32(t*100);
                
                        %ad ogni timestep è necessario tenere traccia della
                        %posizione del robot e del pedone che abbiamo
                        %sostituito

                        if t==simulation_time(k,1)
                            %è il primo timestep del ciclo per cui setto goal e start
                            %position del robot
                            info_index=find(robot.start_info(:,1)==t);
                            robot.set_start_pos(info_index);
                            robot.set_goal(info_index);
                        end

                        if(ped_list(ped_to_skip).trajectory(index,2:3)) ~= [-99999999/1000 -99999999/1000]
                            pos_pedone=ped_list(ped_to_skip).trajectory(index,2:3);
                            pos_robot=robot.curr_pos;
                        else
                            pos_pedone=robot.curr_pos;
                            pos_robot=robot.curr_pos;
                        end
                        
                        if norm(robot.curr_pos-robot.goal_pos)<=0.1
                            %il goal è stato raggiunto
                            robot.V_curr=[0 0];
                            pos_robot=robot.curr_pos;
                        else
                            robot.move_one_step(ped_to_skip,t,time_step);
                        end
                        
                        distance_robot_pedestrian=[distance_robot_pedestrian; pos_robot-pos_pedone];
                        
%                         pos_curr=robot.curr_pos
%                         V_curr=robot.V_curr
%                         e=robot.e
%                         n_ped=robot.n_ped
%                         n_obs=robot.n_obs
%                         n_wall=robot.n_wall
%                         F_obs=robot.F_rep_obs
%                         F_ped=robot.F_rep_ped_tot
%                         F_wall=robot.F_rep_wall
%                         F_att=robot.F_goal
%                         F_tot=robot.F_tot

                        if(anynan(distance_robot_pedestrian))
                            distance_robot_pedestrian
                        end

                    end
                end 
            end
            %finita la simulazione per tutti i pedoni da skippare passo a
            %valutare la fitness function per il singolo cromosoma
            if(anynan(distance_robot_pedestrian))
                distance_robot_pedestrian
            end
            fitness_values(chromosome_index)= evaluate_fitness(distance_robot_pedestrian);
        end
        %sulla base dei valori ricavati di fitness si esegue la selection
        %tra target_vector X e trial_vector U
        [min_fitness_value, min_value_index]=min(fitness_values);
        new_population=[new_population possible_chromosome(min_value_index)];
        
        %salvo il cromosoma se determina delle performance migliori
        if (min_fitness_value < best_fitness_value)
            best_fitness_value=min_fitness_value;
            best_chromosome = possible_chromosome(min_value_index);
        end
        
        if(min_fitness_value < best_fitness_value_of_curr_generation)
            best_fitness_value_of_curr_generation=min_fitness_value;
        end

    end
    population=new_population;
    plot_fitness_values_matrix(n_gen,:)=[n_gen best_fitness_value_of_curr_generation];
    
end

%PLOTTING DEI FITNESS VALUES DURANTE LE GENERAZIONI
figure()
hold on
plot(plot_fitness_values_matrix(:,1),plot_fitness_values_matrix(:,2))
ylabel("Fitness Value")
xlabel("Generation")


%% PLOTTING CON I BEST PARAMETRI FINALI

%INFO GOAL
Goal_1=[4.440, 8.550];
Goal_2=[9.030, 3.580];
Goal_3=[1.110, -3.447];
Goal_4=[-5.468, -6.159];
Goal_5=[-0.130, 4.150];

Goals=[Goal_1; Goal_2; Goal_3; Goal_4; Goal_5];
goal_string="Goal";

figure()
hold on
axis equal
axis([-8,10,-8,10])
legend('Obstacle','','','','','','Helmet 1','Helmet 2','Helmet 3','Helmet 4','Helmet 5','Helmet 6','Helmet 7','Helmet 8','Helmet 9','Helmet 10','Velodyne','Citi','Location','northwest')
hold on

for k=1:length(simulation_time)
    k;

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
                if pos~=[-99999999/1000 -99999999/1000]
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
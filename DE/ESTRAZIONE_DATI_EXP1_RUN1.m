clear
close all
clc

%% CARICHIAMO IL DATASET RELATIVO ALL'ESPERIMENTO 3

load('Ex_1_run_1')
% load('Ex_1_run_2')
% load('Ex_1_run_3')
% load('Ex_1_run_4')


%% RICOSTRUZIONE TRAIETTORIE IN UNA NUOVA TABELLA
Frame=zeros([size(Experiment_1_run_1_0050.RigidBodies.Positions,3) 1]);
Time=zeros([size(Experiment_1_run_1_0050.RigidBodies.Positions,3) 1]);
X=zeros([size(Experiment_1_run_1_0050.RigidBodies.Positions,3) 13]);
Y=zeros([size(Experiment_1_run_1_0050.RigidBodies.Positions,3) 13]);
data_1_1=table(Frame,Time, X, Y);


%copio e incollo colonne "frame" e "time" dalla vecchia tabella
%dataset.Frame(:)=data.Frame(:);
%dataset.Time(:)=data.Time(:);
data_1_1.Frame=linspace(Experiment_1_run_1_0050.StartFrame,Experiment_1_run_1_0050.Frames,Experiment_1_run_1_0050.Frames)';
data_1_1.Time=data_1_1.Frame/Experiment_1_run_1_0050.FrameRate;


for i=1:13
    data_1_1.X(:,i)=Experiment_1_run_1_0050.RigidBodies.Positions(i,1,:);
    data_1_1.Y(:,i)=Experiment_1_run_1_0050.RigidBodies.Positions(i,2,:);
end

writetable(data_1_1,"exp_1_run_1.csv")

% %% PLOTTAGGIO DATI
% figure
% hold on
% axis equal
% axis([-8000,10000,-6000,10000])
% legend('Obstacle','Helmet 1','Helmet 2','Helmet 3','Helmet 4','Helmet 5','Helmet 6','Helmet 7','Helmet 8','Helmet 9','Helmet 10','Velodyne','Citi','Location','northwest')
% frame_counter = 1;
% 
% frame_skip=50;
% 
% Goal_1=[4440, 8550];
% Goal_2=[9030, 3580];
% Goal_3=[1110, -3447];
% Goal_4=[-5468, -6159];
% Goal_5=[-130, 4150];
% 
% Goals=[Goal_1; Goal_2; Goal_3; Goal_4; Goal_5];
% hold on
% goal_string="Goal";
% skip_frame=100;
% 
% for t=1:skip_frame:data_1_1.Frame(end)
%     plotMap(obst_x,obst_y,1)
%     rectangle('Position',[1800 -5600 7800 1200],'FaceColor',[1 1 1])
%     text(2000, -5000, strcat('Frame-', int2str(t)),'FontWeight','bold','FontSize',16,'Color',[0,0,0])
% 
%     for k=1:length(Goals)
%         plot(Goals(k,1),Goals(k,2)+100,".","MarkerSize",30,"Color","r");
%         text(Goals(k,1),Goals(k,2)+100,strcat(goal_string,int2str(k)),"Color", "r")
%     end
% 
%     for ped=2:13
%         pos = [data_1_1.X(t,ped) data_1_1.Y(t,ped)];
%         if(isnan(pos))
%             pos = [19999,19999];
%         end
%         plot(pos(1),pos(2),'.','MarkerSize',30,'Color',body_colors(ped,:))
%         if(ped<12)
%             text(pos(1),pos(2),int2str(ped-1))
%         end
%         hold on
%     end
% 
%     legend('Obstacle','Helmet 1','Helmet 2','Helmet 3','Helmet 4','Helmet 5','Helmet 6','Helmet 7','Helmet 8','Helmet 9','Helmet 10','Velodyne','Citi','Location','northwest')
%     M(frame_counter) = getframe;
%     frame_counter = frame_counter + 1;
%     pause(0.000001)
%     clf
%     hold on
%     axis equal
%     axis([-8000,10000,-6000,10000])
% end
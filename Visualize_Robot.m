function [] = Visualize_Robot(Tleg,T1, T2, T3, T4, T5, T6, color, flag)
%VISUALIZE_ROBOT Summary of this function goes here
%   Detailed explanation goes here
limit = 1.3;
%% Plotting
hold on
line([-limit limit],[0 0],[0 0],'Color','k','LineStyle','--')
line([0 0],[0 0],[-limit limit],'Color','k','LineStyle','--')
line([0 0],[-limit limit],[0 0],'Color','k','LineStyle','--')

config_joint_1 = T1;
config_joint_2 = T2;
config_joint_3 = T3;
config_joint_4 = T4;
config_joint_5 = T5;
config_joint_6 = T6;
config_joint_7 = Tleg;

Tbase = eye(4);

if flag == 1
line([Tbase(1,4) config_joint_1(1,4)],[Tbase(2,4) config_joint_1(2,4)],[Tbase(3,4) config_joint_1(3,4)],...
    'linewidth', 2,'Color',color, 'LineStyle', '--');
line([config_joint_1(1,4) config_joint_2(1,4)],[config_joint_1(2,4) config_joint_2(2,4)],...
    [config_joint_1(3,4) config_joint_2(3,4)],'linewidth', 1,'Color',color, 'LineStyle', '--')
line([config_joint_2(1,4) config_joint_3(1,4)],[config_joint_2(2,4) config_joint_3(2,4)],...
    [config_joint_2(3,4) config_joint_3(3,4)],'linewidth', 1,'Color',color, 'LineStyle', '--')
line([config_joint_3(1,4) config_joint_4(1,4)],[config_joint_3(2,4) config_joint_4(2,4)],...
    [config_joint_3(3,4) config_joint_4(3,4)],'linewidth', 1,'Color',color, 'LineStyle', '--')
line([config_joint_4(1,4) config_joint_5(1,4)],[config_joint_4(2,4) config_joint_5(2,4)],...
    [config_joint_4(3,4) config_joint_5(3,4)],'linewidth', 1,'Color',color, 'LineStyle', '--')
line([config_joint_5(1,4) config_joint_6(1,4)],[config_joint_5(2,4) config_joint_6(2,4)],...
    [config_joint_5(3,4) config_joint_6(3,4)],'linewidth', 1,'Color',color)
line([config_joint_6(1,4) config_joint_7(1,4)],[config_joint_6(2,4) config_joint_7(2,4)],...
    [config_joint_6(3,4) config_joint_7(3,4)],'linewidth', 2,'Color',color)

% plot3(0,0,0,'r*','linewidth', 4,'MarkerSize',10)
plot3(Tbase(1,4),Tbase(2,4),Tbase(3,4),'r*','linewidth', 2,'MarkerSize', 2)
plot3(config_joint_1(1,4),config_joint_1(2,4),config_joint_1(3,4),'r*','linewidth', 2,'MarkerSize',2)
plot3(config_joint_2(1,4),config_joint_2(2,4),config_joint_2(3,4),'r*','linewidth', 2,'MarkerSize',2)
plot3(config_joint_3(1,4),config_joint_3(2,4),config_joint_3(3,4),'r*','linewidth', 2,'MarkerSize',2)
plot3(config_joint_4(1,4),config_joint_4(2,4),config_joint_4(3,4),'r*','linewidth', 2,'MarkerSize',2)
plot3(config_joint_5(1,4),config_joint_5(2,4),config_joint_5(3,4),'r*','linewidth', 2,'MarkerSize',2)
plot3(config_joint_6(1,4),config_joint_6(2,4),config_joint_6(3,4),'r*','linewidth', 2,'MarkerSize',3)
plot3(config_joint_7(1,4),config_joint_7(2,4),config_joint_7(3,4),'r*','linewidth', 1,'MarkerSize',3)

else 
line([Tbase(1,4) config_joint_1(1,4)],[Tbase(2,4) config_joint_1(2,4)],[Tbase(3,4) config_joint_1(3,4)],...
    'linewidth', 2,'Color',color);
line([config_joint_1(1,4) config_joint_2(1,4)],[config_joint_1(2,4) config_joint_2(2,4)],...
    [config_joint_1(3,4) config_joint_2(3,4)],'linewidth', 2,'Color',color)
line([config_joint_2(1,4) config_joint_3(1,4)],[config_joint_2(2,4) config_joint_3(2,4)],...
    [config_joint_2(3,4) config_joint_3(3,4)],'linewidth', 2,'Color',color)
line([config_joint_3(1,4) config_joint_4(1,4)],[config_joint_3(2,4) config_joint_4(2,4)],...
    [config_joint_3(3,4) config_joint_4(3,4)],'linewidth', 2,'Color',color)
line([config_joint_4(1,4) config_joint_5(1,4)],[config_joint_4(2,4) config_joint_5(2,4)],...
    [config_joint_4(3,4) config_joint_5(3,4)],'linewidth', 2,'Color',color)
line([config_joint_5(1,4) config_joint_6(1,4)],[config_joint_5(2,4) config_joint_6(2,4)],...
    [config_joint_5(3,4) config_joint_6(3,4)],'linewidth', 2,'Color',color)
line([config_joint_6(1,4) config_joint_7(1,4)],[config_joint_6(2,4) config_joint_7(2,4)],...
    [config_joint_6(3,4) config_joint_7(3,4)],'linewidth', 2,'Color',color)

% plot3(0,0,0,'r*','linewidth', 4,'MarkerSize',10)
plot3(Tbase(1,4),Tbase(2,4),Tbase(3,4),'r*','linewidth', 2,'MarkerSize', 3)
plot3(config_joint_1(1,4),config_joint_1(2,4),config_joint_1(3,4),'r*','linewidth', 2,'MarkerSize',3)
plot3(config_joint_2(1,4),config_joint_2(2,4),config_joint_2(3,4),'r*','linewidth', 2,'MarkerSize',3)
plot3(config_joint_3(1,4),config_joint_3(2,4),config_joint_3(3,4),'r*','linewidth', 2,'MarkerSize',3)
plot3(config_joint_4(1,4),config_joint_4(2,4),config_joint_4(3,4),'r*','linewidth', 2,'MarkerSize',3)
plot3(config_joint_5(1,4),config_joint_5(2,4),config_joint_5(3,4),'r*','linewidth', 2,'MarkerSize',3)
plot3(config_joint_6(1,4),config_joint_6(2,4),config_joint_6(3,4),'r*','linewidth', 2,'MarkerSize',4)
plot3(config_joint_7(1,4),config_joint_7(2,4),config_joint_7(3,4),'r*','linewidth', 2,'MarkerSize',4)
    
end
end




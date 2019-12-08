folder_set = {'move1','move2','move3'};
map_set = {'map2','map3','map4'};
prefix_set = {'high_vis'};
dir_base = '/home/jbs/catkin_ws/src/traj_gen_vis/log';
addpath(genpath(dir_base));

fig_h = figure(1);
fig_h.Position = [207 118 1565 832];
i = 1;

map_idx = 1;
for map = map_set
    folder_idx = 1;
    for folder = folder_set 
        for prefix = prefix_set
            dir = strcat(dir_base,'/',map,'/',prefix,'/',folder,'/');
            dir = dir{1};
            
            load(strcat(dir,'bearing_clutter.txt'))
            load(strcat(dir,'target_SEDT.txt'))
            load(strcat(dir,'chaser_SEDT.txt'))
            load(strcat(dir,'prediction_error_history.txt'))
            ts = linspace(0,length(bearing_clutter)*0.03,length(bearing_clutter)-1);
            ts_pred = linspace(0,length(bearing_clutter)*0.03,length(prediction_error_history)-1);
            subplot(3,3,i)
            title(strcat('map',int2str(map_idx),'/','case',int2str(folder_idx)))
            hold on 
            plot(ts,bearing_clutter(2:end),'k-','LineWidth',3)
            h_err = plot(ts_pred,smooth(prediction_error_history(2:end),100),'g-','LineWidth',2);
            h_err.Color(4) = 0.5;
            plot(ts,smooth(target_SEDT(2:end),50),'-.','Color',[255 0 127]/255,'LineWidth',1.8);
            plot(ts,smooth(chaser_SEDT(2:end),50),'-','Color',[0 85 255]/255,'LineWidth',1.8)            
            ylabel('[m]')
            xlabel('[s]')
            i = i + 1;
            hold off
        end    
        folder_idx = folder_idx + 1;
    end
    h_lgd = legend({'bearing','pred. err.','target EDT','chaser EDT'});
    h_lgd.FontSize = 10;
    map_idx = map_idx +1;
end

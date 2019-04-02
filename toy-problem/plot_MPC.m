function plot_MPC(opt,cost,t_obs,x_min)
%PLOT_MPC Summary of this function goes here

    % colors
    grass  = .7*[.15 1 .15];
    PH_nom = [0 0 1];
    rect   = [1 0 0];

    % stepping variables
    curI       = 1;
    maxI       = length(opt);
    arrow_step = 1;
    plot_data  = [];
    plot_idx   = 1;
    maxT       = 8;
    
    % closed loop variables
    t = nan(1,maxI);
    x = nan(1,maxI);
    u = nan(1,maxI);
    for i = 1:maxI
        t(i) = opt(i).t(1);
        x(i) = opt(i).x(1);
        u(i) = opt(i).u(1);
    end
        
    % setup figure
    f = figure('WindowKeyPressFcn', @keypress_callback);
%     set(gcf,'color','k');

    %% initial plotting
    subplot(2,2,1); hold on;
        % draw road
        rectangle('Position',[t_obs,-10,1,11],'FaceColor','r');
%         obs_verts = [obs.s_min e_min; obs.s_max e_min; obs.s_max obs.e_min; obs.s_min obs.e_min];
%         faces     = [1 2 3 4];
%         patch('Faces',faces,'Vertices',obs_verts,'FaceColor',rect, ...
%               'EdgeColor','none','FaceAlpha',.7)
        
        % draw closed loop
        plot(t, x);
        
        % draw prediction horizon
        plot_data(plot_idx) = scatter(opt(curI).t, opt(curI).x, ...
                                      'MarkerEdgeColor', PH_nom, ...
                                      'LineWidth',2);
        plot_idx = plot_idx + 1;
        
        % customize
        ylabel('time')
        xlabel('position')
        xlim([0,maxT])
        ylim([-1.5 1.5])
        

%     subplot(2,2,2); hold on;
%         plot(t(1:length(Fy)),Fy)
%         plot_data(plot_idx) = scatter(opt(curI).t(1:length(opt(curI).Fy)), opt(curI).Fy);
%             plot_idx = plot_idx + 1;
%         xlabel('time [sec]')
%         ylabel('[kN] lat. force')
%         xlim([0,t(length(Fy))])

    subplot(2,2,3); hold on;
        plot(t,u);
        plot_data(plot_idx) = scatter(opt(curI).t, opt(curI).u, ...
                                     'MarkerEdgeColor', PH_nom, ...
                                     'LineWidth',2);
            plot_idx = plot_idx + 1;
        xlabel('time')
        ylabel('input')
        xlim([0,maxT])

    subplot(2,2,4); hold on;
% %         plot(t(1:length([cost.sum])),[cost.sum])
%         bar(t(1:length([cost.sum])),[[cost.e]',[cost.Fy]'], ...
%             'stacked','EdgeColor','none')
%         plot_data(plot_idx) = plot([t(curI),t(curI)],[0,max([cost.sum])],'Color',.5*[1 1 1]);
%             plot_idx = plot_idx + 1;
%         title('MPC costs')
%         xlabel('time [sec]')
%         ylabel(' ')
%         legend('lateral error','lateral force')

    %% keypress advance
    function keypress_callback(~, event)
        if strcmp(event.Key, 'rightarrow')
            increment_I(arrow_step);
        end
        if strcmp(event.Key, 'leftarrow')
            decrement_I(arrow_step);
        end
        update_axes();
    end

    function increment_I(step)
        curI = curI + step;
        if curI > maxI
            curI = maxI;
        end
    end

    function decrement_I(step)
        curI = curI - step;
        if curI < 1
            curI = 1;
        end
    end

    function update_axes()
        plot_idx = 1;
        
        set(plot_data(plot_idx), 'XData', opt(curI).t, ...
                                 'YData', opt(curI).x);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', opt(curI).t, ...
                                 'YData', opt(curI).u);
            plot_idx = plot_idx + 1;
%         set(plot_data(plot_idx), 'XData', opt(curI).t(1:length(opt(curI).Uy)), ...
%                                  'YData', opt(curI).Uy);
%             plot_idx = plot_idx + 1;
%         set(plot_data(plot_idx), 'XData', [t(curI),t(curI)], ...
%                                  'YData', [0,max([cost.sum])]);
%             plot_idx = plot_idx + 1;
    end

end

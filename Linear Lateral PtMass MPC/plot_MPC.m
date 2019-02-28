function plot_MPC(t,s,e,Fy,Uy,e_min,e_max,obs,opt,cost)
%PLOT_MPC Summary of this function goes here

    % colors
    grass  = .7*[.15 1 .15];
    PH_nom = [0 0 1];
    rect   = [1 0 0];

    % stepping variables
    curI = 1;
    maxI = length(opt);
    arrow_step = 1;
    plot_data = [];
    plot_idx  = 1;
    
    % setup figure
    f = figure('WindowKeyPressFcn', @keypress_callback);
%     set(gcf,'color','k');

    %% initial plotting
    subplot(2,2,1); hold on; axis equal;
        % draw road
        set(gca,'Color',.35*[1 1 1])
        h = 8;
        rectangle('Position',[0, e_min-h, max(s(1:length(e))), h], ...
                  'FaceColor',grass,'EdgeColor','none');
        rectangle('Position',[0, e_max,   max(s(1:length(e))), h], ...
                  'FaceColor',grass,'EdgeColor','none');
        plot(s, zeros(size(s)),'--y','LineWidth',2);
        plot(s, e_min*ones(size(s)),'Color',.5*[1 1 1])
        plot(s, e_max*ones(size(s)),'Color',.5*[1 1 1])
        obs_verts = [obs.s_min e_min; obs.s_max e_min; obs.s_max obs.e_min; obs.s_min obs.e_min];
        faces     = [1 2 3 4];
        patch('Faces',faces,'Vertices',obs_verts,'FaceColor',rect, ...
              'EdgeColor','none','FaceAlpha',.7)
        % plot states & PH
        plot(s(1:length(e)), e);
        plot_data(plot_idx) = scatter(opt(curI).s, opt(curI).e, ...
                                      'MarkerEdgeColor', PH_nom, ...
                                      'LineWidth',2);
            plot_idx = plot_idx + 1;
        % customize
        ylabel('lateral position [m]')
        xlabel('longitudinal position [m]')
        xlim([0,max(s(1:length(e)))])
        ylim([-8 8])
        
        

    subplot(2,2,2); hold on;
        plot(t(1:length(Fy)),Fy)
        plot_data(plot_idx) = scatter(opt(curI).t(1:length(opt(curI).Fy)), opt(curI).Fy);
            plot_idx = plot_idx + 1;
        xlabel('time [sec]')
        ylabel('[kN] lat. force')
        xlim([0,t(length(Fy))])

    subplot(2,2,3); hold on;
        plot(t(1:length(Uy)),Uy)
        plot_data(plot_idx) = scatter(opt(curI).t(1:length(opt(curI).Uy)), opt(curI).Uy);
            plot_idx = plot_idx + 1;
        xlabel('time [sec]')
        ylabel('lat. speed')
        xlim([0,t(length(Uy))])

    subplot(2,2,4); hold on;
%         plot(t(1:length([cost.sum])),[cost.sum])
        bar(t(1:length([cost.sum])),[[cost.e]',[cost.Fy]'], ...
            'stacked','EdgeColor','none')
        plot_data(plot_idx) = plot([t(curI),t(curI)],[0,max([cost.sum])],'Color',.5*[1 1 1]);
            plot_idx = plot_idx + 1;
        title('MPC costs')
        xlabel('time [sec]')
        ylabel(' ')
        legend('lateral error','lateral force')

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
        
        set(plot_data(plot_idx), 'XData', opt(curI).s, ...
                                 'YData', opt(curI).e);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', opt(curI).t(1:length(opt(curI).Fy)), ...
                                 'YData', opt(curI).Fy);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', opt(curI).t(1:length(opt(curI).Uy)), ...
                                 'YData', opt(curI).Uy);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', [t(curI),t(curI)], ...
                                 'YData', [0,max([cost.sum])]);
            plot_idx = plot_idx + 1;
    end

end

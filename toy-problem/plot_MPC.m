function plot_MPC(opt,cost,t_obs,x_min)
%PLOT_MPC Summary of this function goes here

    % colors
    grass  = .7*[.15 1 .15];                        % green
    PH_nom = [0 0 1];                               % blue
    rect   = [1 0 0];                               % red

    curI       = 1;    maxI      = length(opt);     % stepping variables
    arrow_step = 1;    plot_data = [];
    plot_idx   = 1;    maxT      = 8;
    
    t = nan(1,maxI);                                % closed loop variables
    x = nan(1,maxI);
    u = nan(1,maxI);
    for i = 1:maxI
        t(i) = opt(i).t(1);
        x(i) = opt(i).x(1);
        u(i) = opt(i).u(1);
    end
        
    f = figure('WindowKeyPressFcn', @keypress_callback); % setup figure
%     set(gcf,'color','k');

    % initial plotting
    %% robust
    subplot(3,3,1); hold on;                        % robust states
        obs_verts = [t_obs -10; t_obs 1; ...        % draw obstacle
                     t_obs+1 1; t_obs+1 -10];
        faces     = [1 2 3 4];
        patch('Faces',faces,'Vertices',obs_verts,'FaceColor',rect, ...
              'EdgeColor','r','LineStyle','--','LineWidth',2,'FaceAlpha',.3)
        plot(t, x);                                 % draw closed loop
        plot_data(plot_idx) = ...
            scatter(opt(curI).t, opt(curI).x, ...   % draw PH
            'MarkerEdgeColor', PH_nom, 'LineWidth',2);
            plot_idx = plot_idx + 1;
        title('Robust MPC');                        % customize
        xlabel('time');     ylabel('position'); 
        xlim([0,maxT]);     ylim([-1.5 1.5]);

    subplot(3,3,4); hold on;                        % robust inputs
        plot(t,u);                                  % draw closed loop
        plot_data(plot_idx) = ...                   % draw PH
            scatter(opt(curI).t(1:end-1), opt(curI).u, ...
            'MarkerEdgeColor', PH_nom, 'LineWidth',2);
            plot_idx = plot_idx + 1;
        xlabel('time'); ylabel('input');            % customize
        xlim([0,maxT]);
        
    subplot(3,3,7); hold on;                        % robust costs
        cumcost = cumsum([cost.sum]);
        plot(t,cumcost)
        bar(t,[cost.u_n],'stacked','EdgeColor','none')
        plot_data(plot_idx) = plot([t(curI),t(curI)],[0,max(cumcost)],'Color',.5*[1 1 1]);
            plot_idx = plot_idx + 1;
        xlabel('time'); %ylabel('')
        legend('input cost')
        

    %% pure contingency
    subplot(3,3,2); hold on;                        % contingency states
        title('Contingency MPC');                   % customize
    subplot(3,3,5); hold on;                        % contingency inputs
    subplot(3,3,8); hold on;                        % contingency costs
    
    %% mixed results
    subplot(3,3,2); hold on;                        %
        title('Mixed CMPC');                        % customize
    subplot(3,3,5); hold on;                        %
    subplot(3,3,8); hold on;                        %


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
        set(plot_data(plot_idx), 'XData', opt(curI).t, ...          % robust states
                                 'YData', opt(curI).x);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', opt(curI).t(1:end-1), ... % robust inputs
                                 'YData', opt(curI).u);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', [t(curI),t(curI)], ...    % robust cost
                                 'YData', [0,max(cumcost)]);
            plot_idx = plot_idx + 1;
    end

end

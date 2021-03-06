function plot_MPC(opt_r,cost_r,opt_50,cost_50,opt_25,cost_25,opt_c,cost_c,t_obs,x_min)
%PLOT_MPC Summary of this function goes here

    % colors
    c_grass = 0.7*[0.15 1 0.15];                     % green
    c_nom   =     [0    0 1.0 ];                     % blue
    c_50    =     [0.3  0 0.7 ];                     % purpl-ish
    c_25    =     [0.7  0 0.3 ];                     % purpl-ish
    c_cont  =     [1.0  0 0.0 ];                     % red
    c_rect  =     [1    0 0   ];                     % red

    curI       = 1;    maxI      = length(opt_r);   % stepping variables
    arrow_step = 1;    plot_data = [];
    plot_idx   = 1;    maxT      = 9;
    
    t_r  = nan(1,maxI);                              % closed loop variables
    x_r  = nan(1,maxI);
    u_r  = nan(1,maxI);
    t_50 = nan(1,maxI);
    x_50 = nan(1,maxI);
    u_50 = nan(1,maxI);
    t_25 = nan(1,maxI);
    x_25 = nan(1,maxI);
    u_25 = nan(1,maxI);
    t_c  = nan(1,maxI);
    x_c  = nan(1,maxI);
    u_c  = nan(1,maxI);
    for i = 1:maxI
        t_r(i)  = opt_r(i).t(1);
        x_r(i)  = opt_r(i).x(1);
        u_r(i)  = opt_r(i).u(1);
        t_50(i) = opt_50(i).t(1);
        x_50(i) = opt_50(i).x_c(1);
        u_50(i) = opt_50(i).u_c(1);
        t_25(i) = opt_25(i).t(1);
        x_25(i) = opt_25(i).x_c(1);
        u_25(i) = opt_25(i).u_c(1);
        t_c(i)  = opt_c(i).t(1);
        x_c(i)  = opt_c(i).x_c(1);
        u_c(i)  = opt_c(i).u_c(1);
    end
        
    f = figure( 'WindowKeyPressFcn', ...
                @keypress_callback );               % setup figure
%     set(gcf,'color','k');

    % initial plotting
    %% robust
    subplot(3,3,1); hold on;                        % robust states
        obs_verts = [t_obs -10; t_obs 1; ...        %   draw obstacle
                     t_obs+1 1; t_obs+1 -10];
        faces     = [1 2 3 4];
        patch('Faces',faces,'Vertices',obs_verts,'FaceColor',c_rect, ...
              'EdgeColor','r','LineStyle','--','LineWidth',2,'FaceAlpha',.3)
        plot(t_r, x_r, 'Color', c_nom);            %   draw closed loop
        plot_data(plot_idx) = ...
            scatter(opt_r(curI).t, opt_r(curI).x,...%   draw PH
            'MarkerEdgeColor', c_nom, 'LineWidth',2);
            plot_idx = plot_idx + 1;
        title('Robust MPC');                        %   customize
        xlabel('time');     ylabel('position'); 
        xlim([0,maxT]);     ylim([-1.5 1.5]);

    subplot(3,3,4); hold on;                        % robust inputs
        plot(t_r, u_r, 'Color', c_nom);            %   draw closed loop
        plot_data(plot_idx) = ...                   %   draw PH
            scatter(opt_r(curI).t(1:end-1), opt_r(curI).u, ...
            'MarkerEdgeColor', c_nom, 'LineWidth',2);
            plot_idx = plot_idx + 1;
        xlabel('time'); ylabel('input');            %   customize
        xlim([0,maxT]); ylim([-0.5 1.5]);
        
    subplot(3,3,7); hold on;                        % robust costs
        cumcost_r = cumsum([cost_r.sum]);
        bar(t_r,[cost_r.u],'stacked','EdgeColor','none')
        plot_data(plot_idx) = plot([t_r(curI),t_r(curI)],[0,max(cumcost_r)],'Color',.5*[1 1 1]);
            plot_idx = plot_idx + 1;
        plot(t_r, cumcost_r, 'Color', c_nom)
        xlabel('time'); %ylabel('')
        xlim([0,maxT]); ylim([0 1.5]);
        legend('cum','input cost')
        

    %% pure contingency
    subplot(3,3,2); hold on;                        % contingency states
        obs_verts = [t_obs -10; t_obs 1; ...        %   draw obstacle
                     t_obs+1 1; t_obs+1 -10];
        faces     = [1 2 3 4];
        patch('Faces',faces,'Vertices',obs_verts,'FaceColor',c_rect, ...
              'EdgeColor','r','LineStyle','--','LineWidth',2,'FaceAlpha',.3)
        plot(t_c, x_c, 'Color', c_cont);           %   draw closed loop
        plot_data(plot_idx) = ...                   %   draw cont PH
            scatter(opt_c(curI).t, opt_c(curI).x_c, ...
            'MarkerEdgeColor', c_cont, 'LineWidth',3);
            plot_idx = plot_idx + 1;
        plot_data(plot_idx) = ...                   %   draw nom PH
            scatter(opt_c(curI).t, opt_c(curI).x_n, ...
            'MarkerEdgeColor', c_nom, 'LineWidth',2);
            plot_idx = plot_idx + 1;
        title('Contingency MPC');                   %   customize
        xlabel('time');     ylabel('position'); 
        xlim([0,maxT]);     ylim([-1.5 1.5]);
    subplot(3,3,5); hold on;                        % contingency inputs
        plot(t_c, u_c, 'Color', c_cont);           %   draw closed loop
        plot_data(plot_idx) = ...                   %   draw cont PH
            scatter(opt_c(curI).t(1:end-1), opt_c(curI).u_c, ...
            'MarkerEdgeColor', c_cont, 'LineWidth',2);
            plot_idx = plot_idx + 1;
        plot_data(plot_idx) = ...                   %   draw nom PH
            scatter(opt_c(curI).t(1:end-1), opt_c(curI).u_n, ...
            'MarkerEdgeColor', c_nom, 'LineWidth',1);
            plot_idx = plot_idx + 1;
        xlabel('time'); ylabel('input');            %   customize
        xlim([0,maxT]); ylim([-0.5 1.5]);
    subplot(3,3,8); hold on;                        % contingency costs
        cumcost_c = cumsum([cost_c.sum]);
        bar(t_c,[[cost_c.u_n]',[cost_c.u_c]'],'stacked','EdgeColor','none')
        plot_data(plot_idx) = plot([t_c(curI),t_c(curI)],[0,max(cumcost_c)],'Color',.5*[1 1 1]);
            plot_idx = plot_idx + 1;
        plot(t_c, cumcost_c, 'Color', c_cont)
        xlabel('time'); %ylabel('')                 %   customize
        xlim([0,maxT]); ylim([0 1.5]);
        legend('cum','u_n cost','u_c cost')
    
    %% mixed results
    subplot(3,3,3); hold on;                        % mixed states
        obs_verts = [t_obs -10; t_obs 1; ...        %   draw obstacle
                     t_obs+1 1; t_obs+1 -10];
        faces     = [1 2 3 4];
        patch('Faces',faces,'Vertices',obs_verts,'FaceColor',c_rect, ...
              'EdgeColor','r','LineStyle','--','LineWidth',2,'FaceAlpha',.3)
                                                    %   draw CL:
        plot(t_r,  x_r,  'Color', c_nom, ...        %     robust
             'LineWidth', 2);
        plot(t_50, x_50, 'Color', c_50,   ...       %     1/2 P
             'LineWidth', 2);
        plot(t_25, x_25, 'Color', c_25,   ...       %     1/4 P
             'LineWidth', 2);
        plot(t_c,  x_c,  'Color', c_cont,...        %     cmpc
             'LineWidth', 2);
        title('Mixed CMPC');                        %   customize
        xlabel('time');     ylabel('position'); 
        xlim([0,maxT]);     ylim([-1.5 1.5]);
    subplot(3,3,6); hold on;                        % mixed inputs
                                                    %   draw CL:
        plot(t_r,  u_r,  'Color', c_nom);           %     robust
        plot(t_50, u_50, 'Color', c_50);            %     1/2 P
        plot(t_25, u_25, 'Color', c_25);            %     1/4 P
        plot(t_c,  u_c,  'Color', c_cont);          %     cmpc
        xlabel('time'); ylabel('input');            %   customize
        xlim([0,maxT]); ylim([-0.5 1.5]);
    subplot(3,3,9); hold on;                        % mixed costs
        cumcost_50 = cumsum([cost_50.sum]);         %   construct cost array
        cumcost_25 = cumsum([cost_25.sum]);
        plot(t_r,  cumcost_r, 'Color', c_nom)       %   plot cumulative costs
        plot(t_50, cumcost_50, 'Color', c_50)
        plot(t_25, cumcost_25, 'Color', c_25)
        plot(t_c,  cumcost_c, 'Color', c_cont)
        xlabel('time'); %ylabel('')                 %   customize
        xlim([0,maxT]); ylim([0 1.5]);
        legend('100% P','50% P','25% P','0% P')

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
        set(plot_data(plot_idx), 'XData', opt_r(curI).t, ...          % robust states
                                 'YData', opt_r(curI).x);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', opt_r(curI).t(1:end-1), ... % robust inputs
                                 'YData', opt_r(curI).u);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', [t_r(curI),t_r(curI)], ...  % robust cost
                                 'YData', [0,max(cumcost_r)]);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', opt_c(curI).t, ...          % contingency states
                                 'YData', opt_c(curI).x_c);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', opt_c(curI).t, ...          % nominal states
                                 'YData', opt_c(curI).x_n);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', opt_c(curI).t(1:end-1), ... % contingency inputs
                                 'YData', opt_c(curI).u_c);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', opt_c(curI).t(1:end-1), ... % nominal inputs
                                 'YData', opt_c(curI).u_n);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', [t_c(curI),t_c(curI)], ...  % contingency cost
                                 'YData', [0,max(cumcost_c)]);
            plot_idx = plot_idx + 1;
    end

end

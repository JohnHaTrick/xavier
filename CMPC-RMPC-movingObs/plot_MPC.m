function plot_MPC( opt_A,   opt_r, cost_r, opt_50, cost_50, opt_25, ...
                   cost_25, opt_c, cost_c, y_obs )
%PLOT_MPC Summary of this function goes here

    % colors
    c_grass = 0.7*[0.15 1 0.15];                        % green
    c_nom   =     [0    0 1.0 ];                        % blue
    c_50    =     [0.3  0 0.7 ];                        % purpl-ish
    c_25    =     [0.7  0 0.3 ];                        % purpl-ish
    c_cont  =     [1.0  0 0.0 ];                        % red
    c_rect  =     [1    0 0   ];                        % red

    curI       = 1;    maxI      = length(opt_A);       % stepping variables
    arrow_step = 1;    maxT      = opt_A(end).t(1);
    plot_data  = [];   plot_idx  = 1;
    patch_data = [];   patch_idx = 1;
    
    t     = nan(1,maxI);                                % closed loop variables
    y     = nan(1,maxI);
    x_min = [opt_A.x_min];
    o_max = max(x_min);
    x_r   = nan(1,maxI);
    u_r   = nan(1,maxI);
    x_50  = nan(1,maxI);
    u_50  = nan(1,maxI);
    x_25  = nan(1,maxI);
    u_25  = nan(1,maxI);
    x_c   = nan(1,maxI);
    u_c   = nan(1,maxI);
    for i = 1:maxI                                      % fill vars
        t(i)    = opt_A(i).t(1);
        y(i)    = opt_A(i).y(1);
        x_r(i)  = opt_r(i).x(1);
        u_r(i)  = opt_r(i).u(1);
        x_50(i) = opt_50(i).x_c(1);
        u_50(i) = opt_50(i).u_c(1);
        x_25(i) = opt_25(i).x_c(1);
        u_25(i) = opt_25(i).u_c(1);
        x_c(i)  = opt_c(i).x_c(1);
        u_c(i)  = opt_c(i).u_c(1);
    end
        
    f = figure( 'WindowKeyPressFcn', ...                % setup figure
                @keypress_callback );

    x_min_i   = x_min(curI);                            % obstacle params
    verts_i   = [y_obs -10;       y_obs x_min_i; ...    %   vertices for exp. growth
                 y_obs+1 x_min_i; y_obs+1 -10];
    verts_max = [y_obs -10;       y_obs o_max; ...      %   vertices for max growth
                 y_obs+1 o_max;   y_obs+1 -10];
    faces     = [1 2 3 4];
    
    %% robust                                           % initial plotting
    subplot(3,3,1); hold on; axis equal;                % robust states
        patch(    'Faces',faces,'Vertices',verts_max,'FaceColor',c_rect, ...
                  'EdgeColor','r','LineStyle','--','EdgeAlpha',.1,'LineWidth',2,'FaceAlpha',.1)
        patch_data(patch_idx) = ...
            patch('Faces',faces,'Vertices',verts_i,  'FaceColor',c_rect, ...
                  'EdgeColor','r','LineStyle','--','EdgeAlpha',1, 'LineWidth',2,'FaceAlpha',.3);
            patch_idx = patch_idx + 1;
        plot(t, x_r, 'Color', c_nom);                   %   draw closed loop
        plot_data(plot_idx) = ...
            scatter(opt_A(curI).t, opt_r(curI).x,...    %   draw PH
            'MarkerEdgeColor', c_nom, 'LineWidth',2);
            plot_idx = plot_idx + 1;
        title('Robust MPC');                            %   customize
        xlabel('East');     ylabel('North'); 
        xlim([0,maxT]);     ylim(1.2*[-o_max o_max]);

    subplot(3,3,4); hold on;                            % robust inputs
        plot(t, u_r, 'Color', c_nom);                   %   draw closed loop
        plot_data(plot_idx) = ...                       %   draw PH
            scatter(opt_A(curI).t(1:end-1), opt_r(curI).u, ...
            'MarkerEdgeColor', c_nom, 'LineWidth', 2);
            plot_idx = plot_idx + 1;
        xlabel('time'); ylabel('input');                %   customize
        xlim([0,maxT]); ylim([-0.5 1.5]);
        
    subplot(3,3,7); hold on;                            % robust costs
        bar( t,[cost_r.u],'stacked','EdgeColor','none')
        plot(t,[cost_r.cum], 'Color', c_nom, 'LineWidth', 2)
        plot_data(plot_idx) = plot([t(curI),t(curI)], ...
                                   [0,max(max([cost_r.u]),cost_r(end).cum)], ...
                                   'Color',.5*[1 1 1],'LineWidth',2);
        set(get(get(plot_data(plot_idx),'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
            plot_idx = plot_idx + 1;
        xlabel('time'); %ylabel('')
        xlim([0,maxT]); ylim([0 1.1*max(max([cost_r.u]),cost_r(end).cum)]);
        legend('opt cost','incurred cost')
        

    %% pure contingency
    subplot(3,3,2); hold on;                            % contingency states
        patch(    'Faces',faces,'Vertices',verts_max,'FaceColor',c_rect, ...
                  'EdgeColor','r','LineStyle','--','EdgeAlpha',.1,'LineWidth',2,'FaceAlpha',.1)
        patch_data(patch_idx) = ...
            patch('Faces',faces,'Vertices',verts_i,  'FaceColor',c_rect, ...
                  'EdgeColor','r','LineStyle','--','EdgeAlpha',1, 'LineWidth',2,'FaceAlpha',.3);
            patch_idx = patch_idx + 1;
        plot(t, x_c, 'Color', c_cont);                  %   draw closed loop
        plot_data(plot_idx) = ...                       %   draw cont PH
            scatter(opt_A(curI).t, opt_c(curI).x_c, ...
            'MarkerEdgeColor', c_cont, 'LineWidth',3);
            plot_idx = plot_idx + 1;
        plot_data(plot_idx) = ...                       %   draw nom PH
            scatter(opt_A(curI).t, opt_c(curI).x_n, ...
            'MarkerEdgeColor', c_nom, 'LineWidth',2);
            plot_idx = plot_idx + 1;
        title('Contingency MPC');                       %   customize
        xlabel('time');     ylabel('position'); 
        xlim([0,maxT]);     ylim(1.2*[-o_max o_max]);
    subplot(3,3,5); hold on;                            % contingency inputs
        plot(t, u_c, 'Color', c_cont);                  %   draw closed loop
        plot_data(plot_idx) = ...                       %   draw cont PH
            scatter(opt_A(curI).t(1:end-1), opt_c(curI).u_c, ...
            'MarkerEdgeColor', c_cont, 'LineWidth',2);
            plot_idx = plot_idx + 1;
        plot_data(plot_idx) = ...                       %   draw nom PH
            scatter(opt_A(curI).t(1:end-1), opt_c(curI).u_n, ...
            'MarkerEdgeColor', c_nom, 'LineWidth',1);
            plot_idx = plot_idx + 1;
        xlabel('time'); ylabel('input');                %   customize
        xlim([0,maxT]); ylim([-0.5 1.5]);
    subplot(3,3,8); hold on;                            % contingency costs
        bar(t,[cost_c.u_n]','stacked','EdgeColor','none')
        plot(t, [cost_c.cum], 'Color', c_cont, 'LineWidth', 2)
        plot_data(plot_idx) = plot([t(curI),t(curI)], ...
                                   [0,max(max([cost_r.u]),cost_r(end).cum)], ...
                                   'Color',.5*[1 1 1],'LineWidth',2);
        set(get(get(plot_data(plot_idx),'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
            plot_idx = plot_idx + 1;
        xlabel('time'); %ylabel('')                     %   customize
        xlim([0,maxT]); ylim([0 1.1*max(max([cost_r.u]),cost_r(end).cum)]);
        legend('opt cost','incurred cost')        
    
    %% mixed results
    subplot(3,3,3); hold on;                            % mixed states
        patch(    'Faces',faces,'Vertices',verts_max,'FaceColor',c_rect, ...
                  'EdgeColor','r','LineStyle','--','EdgeAlpha',.1,'LineWidth',2,'FaceAlpha',.1)
        patch_data(patch_idx) = ...
            patch('Faces',faces,'Vertices',verts_i,  'FaceColor',c_rect, ...
                  'EdgeColor','r','LineStyle','--','EdgeAlpha',1, 'LineWidth',2,'FaceAlpha',.3);
            patch_idx = patch_idx + 1;
                                                        %   draw CL:
        plot(t,  x_r,  'Color', c_nom, ...              %     robust
             'LineWidth', 2);
        plot(t, x_50, 'Color', c_50,   ...              %     1/2 P
             'LineWidth', 2);
        plot(t, x_25, 'Color', c_25,   ...              %     1/4 P
             'LineWidth', 2);
        plot(t,  x_c,  'Color', c_cont,...              %     cmpc
             'LineWidth', 2);
        title('Mixed CMPC');                            %   customize
        xlabel('time');     ylabel('position'); 
        xlim([0,maxT]);     ylim(1.2*[-o_max o_max]);
    subplot(3,3,6); hold on;                            % mixed inputs
                                                        %   draw CL:
        plot(t, u_r,  'Color', c_nom,  'LineWidth', 2); %     robust
        plot(t, u_50, 'Color', c_50,   'LineWidth', 2); %     1/2 P
        plot(t, u_25, 'Color', c_25,   'LineWidth', 2); %     1/4 P
        plot(t, u_c,  'Color', c_cont, 'LineWidth', 2); %     cmpc
        xlabel('time'); ylabel('input');                %   customize
        xlim([0,maxT]); ylim([-0.5 1.5]);
    subplot(3,3,9); hold on;                            % mixed costs
        plot(t, [cost_r.cum],  'Color', c_nom,  'LineWidth', 2)
        plot(t, [cost_50.cum], 'Color', c_50,   'LineWidth', 2)
        plot(t, [cost_25.cum], 'Color', c_25,   'LineWidth', 2)
        plot(t, [cost_c.cum],  'Color', c_cont, 'LineWidth', 2)
        xlabel('time'); %ylabel('')                     %   customize
        xlim([0,maxT]); ylim([0 1.1*max(max([cost_r.u]),cost_r(end).cum)]);
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
        set(plot_data(plot_idx), 'XData', opt_A(curI).t, ...            % robust states
                                 'YData', opt_r(curI).x);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', opt_A(curI).t(1:end-1), ...   % robust inputs
                                 'YData', opt_r(curI).u);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', [t(curI),t(curI)], ...        % robust cost
                                 'YData', [0,max(max([cost_r.u]),cost_r(end).cum)]);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', opt_A(curI).t, ...            % contingency states
                                 'YData', opt_c(curI).x_c);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', opt_A(curI).t, ...            % nominal states
                                 'YData', opt_c(curI).x_n);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', opt_A(curI).t(1:end-1), ...   % contingency inputs
                                 'YData', opt_c(curI).u_c);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', opt_A(curI).t(1:end-1), ...   % nominal inputs
                                 'YData', opt_c(curI).u_n);
            plot_idx = plot_idx + 1;
        set(plot_data(plot_idx), 'XData', [t(curI),t(curI)], ...        % contingency cost
                                 'YData', [0,max(max([cost_r.u]),cost_r(end).cum)]);
%             plot_idx = plot_idx + 1;
            
        verts = [y_obs -10;           y_obs x_min(curI); ...            % vertices for currI
                 y_obs+1 x_min(curI); y_obs+1 -10];
            patch_idx = 1;
        set(patch_data(patch_idx), 'Vertices', verts);
            patch_idx = patch_idx + 1;
        set(patch_data(patch_idx), 'Vertices', verts);
            patch_idx = patch_idx + 1;
        set(patch_data(patch_idx), 'Vertices', verts);
    end

end

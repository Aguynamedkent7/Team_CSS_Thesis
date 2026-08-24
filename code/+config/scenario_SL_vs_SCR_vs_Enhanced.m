function cfg = scenario_SL_vs_SCR_vs_Enhanced(cfg)
% Showcasing SL vs SCR vs Enhanced SCR

cfg.scn.description = [cfg.scn.description '\nwith comparison of SL vs SCR vs Enhanced SCR track discretization controller'];

vehicle_default = config.vehicle_ST_Liniger(config.base_vehicle(cfg));

% vehicle 1: SL
cfg.scn.vhs{1} = vehicle_default;
cfg.scn.vhs{1}.plot_color = 'r';

% vehicle 2: SCR
cfg.scn.vhs{2} = config.vehicle_SCR(vehicle_default);
cfg.scn.vhs{2}.plot_color = 'b';

% vehicle 3: Enhanced SCR
cfg.scn.vhs{3} = config.vehicle_SCR_enhanced(vehicle_default);
cfg.scn.vhs{3}.plot_color = 'g';
end

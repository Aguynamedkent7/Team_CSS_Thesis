function cfg_vh = vehicle_SCR_enhanced(cfg_vh)
% Enhanced SCR vehicle
if nargin == 0
    cfg_vh = config.base_vehicle(config.config());
end

cfg_vh = config.vehicle_SCR(cfg_vh);

cfg_vh.description = 'Enhanced SCR';

%% Enhancements Phase 11: The Sweet Spot
% R(2,2) = 50 survived the whole track but was 10.20s.
% R(2,2) = 1 crashed.
% Let's find the sweet spot that breaks 10.00s!
cfg_vh.p.R = diag([100.0, 15.0]); 

cfg_vh.p.SCP_iterations = 2; 

cfg_vh.p.relax_terminal = true; 
cfg_vh.p.Q_vel = 0.0; 
cfg_vh.p.W_alpha = 0.0; 
cfg_vh.p.dynamic_slack = true; 
cfg_vh.p.slack_kappa_weight = 0.1; 
cfg_vh.p.strict_track_limits = false; 
end

% Find the newest log file
d = dir('../results/*/log.mat');
[~, idx] = max([d.datenum]);
output_file = fullfile(d(idx).folder, d(idx).name);

fprintf('Loading latest results from: %s\n', output_file);
sim_result = load(output_file);

n_vehicles = numel(sim_result.cfg.scn.vhs);
n_laps = sim_result.cfg.race.n_laps;

fprintf('\n--- Lap Times ---\n');
for i_vehicle = 1:n_vehicles
    fprintf('Vehicle %i (%s):\n', i_vehicle, sim_result.cfg.scn.vhs{i_vehicle}.description);
    for i_lap = 1:n_laps
        correcting_first_lap = (i_lap == 1);
        t_lap = (sum([sim_result.log.vehicles{i_vehicle}.lap_count] == i_lap-1) - correcting_first_lap) * sim_result.cfg.scn.vhs{i_vehicle}.p.dt_controller;
        fprintf('  Lap %i: %.2f s\n', i_lap, t_lap);
    end
end
fprintf('-----------------\n');

function toModelWorkspace(model,data)
% Model is the model name as a string.
% Data is a struct with fields corresponding
% to the variables to be stored
load_system(model)
hws = get_param(bdroot, 'modelworkspace');


% Load additional parameters into workspace
fields = fieldnames(data);
for i = 1:numel(fields)
  hws.assignin(fields{i},data.(fields{i}));
end

end
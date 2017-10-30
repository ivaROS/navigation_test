clear; clc;
%%Inputs%%
numTestPerBarrel = 50;

filename = 'output_10_26.csv';
fp = importfile(filename);
controller_names = unique(fp(:,1))';

state_enums = unique(fp(:,6))';
barrel_numbers = unique(cell2mat(fp(:,3)'));

numTests = length(fp(:,1));
numControllers = length(controller_names);
numStates = length(state_enums);
numBarrels = length(barrel_numbers);

vec_each_barrel = zeros(1, numStates);
%vec_each_barrel = cell(1,numStates);
cell_each_barrel = {};
cell_total_controller = {};


mat_each_controller = zeros(numBarrels, 1 + numStates);
for i = 1 : numBarrels
    mat_each_controller(i,1) = barrel_numbers(i);
end

for i = 1 : numControllers
    cell_total_controller{end + 1} = mat_each_controller;
end

for i = 1 : numTests
    controller_name = fp{i, 1};
    state = fp{i, 6};
    num_barrel = fp{i, 3};
    
    barrel_index = find(barrel_numbers == num_barrel, 1);
    state_index = find(strcmp(state, state_enums));
    controller_index = find(strcmp(controller_name, controller_names));
    cell_total_controller{controller_index}(barrel_index, state_index+1) = cell_total_controller{controller_index}(barrel_index, state_index+1) + 1;
end
T=[];

for i = 1 : numControllers
    temp = table(cell_total_controller{i});

    temp.Properties.VariableNames = controller_names(i);
    %temp.Properties.VariableNames = c;
    T = [T temp]; 
end
  
        

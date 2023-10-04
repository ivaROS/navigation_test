clear; clc;
haha = importfile('data_1_numbarrels_3.csv');
num_test_3_barrels = 199;
num_test_5_barrels = 187;
num_test_7_barrels = 200;
success_count_3_barrels = [0 0 0 0 0];
success_count_5_barrels = [0 0 0 0 0 ];
success_count_7_barrels = [0 0 0 0 0];
for i = 1 : num_test_3_barrels
    filename = strcat('data_', int2str(i), '_numbarrels_3.csv');
    fp = importfile(filename);
    for j = 2 : 2: 10
        result = fp{j, 2}(3);
        if (result == 's')
            success_count_3_barrels(j / 2) = success_count_3_barrels(j / 2) + 1;
        end
    end
end

for i = 1 : num_test_5_barrels
    filename = strcat('data_', int2str(i), '_numbarrels_5.csv');
    fp = importfile(filename);
    for j = 2 : 2: 10
        result = fp{j, 2}(3);
        if (result == 's')
            success_count_5_barrels(j / 2) = success_count_5_barrels(j / 2) + 1;
        end
    end
end


for i = 90 : num_test_7_barrels
    filename = strcat('data_', int2str(i), '_numbarrels_7.csv');
    fp = importfile(filename);
    for j = 2 : 2: 10
        result = fp{j, 2}(3);
        if (result == 's')
            success_count_7_barrels(j / 2) = success_count_7_barrels(j / 2) + 1;
        end
    end
end
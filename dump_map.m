load 'data/gmapping.mat'

f = fopen('data/map.bin', 'w');
fwrite(f, map.data, 'uint8');
fclose(f);

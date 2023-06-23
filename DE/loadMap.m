map=imread("exp1_map.png");
map=map(:,:,1);
map=flip(map);
map=map';

[obst_x,obst_y] = find(map == 0);
obst_x = (obst_x * 10 - 11000)/1000;%7000;
obst_y = (obst_y * 10 - 10000)/1000;%6500;
shuffle = randperm(length(obst_x));
obst_x = obst_x(shuffle);
obst_y = obst_y(shuffle);
dirs    = {'/Users/gregorywayne/Google Drive/Deep Control/',...
    '/Users/gregorywayne/Google Drive/Deep Control/Truck/',...
    '/Users/gregorywayne/Google Drive/Deep Control/minFunc/',...
    '/Users/gregorywayne/Google Drive/Deep Control/Obstacle/'};

addpath(dirs{:});

ms      = '*.m';

files   = dir(strcat(dirs{1},ms));
D       = {files.name};
files   = dir(strcat(dirs{2},ms));
D       = {D{:},files.name};
files   = dir(strcat(dirs{3},ms));
D       = {D{:},files.name};
files   = dir(strcat(dirs{4},ms));
D       = {D{:},files.name};

D       = {D{:},'hcontroller.mat','pforward.mat',...
                'gforward.mat','oforward.mat','ocdecoder.mat',...
                '/Users/gregorywayne/Google Drive/Deep Control/Truck/lcontroller.mat'};
            
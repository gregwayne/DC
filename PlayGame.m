function logging = PlayGame(gameName,env_game)

    close all;
    addpath(gameName);
    global env;
    env     = env_game;
    rng('shuffle');
    logging = GameMain(gameName);

end
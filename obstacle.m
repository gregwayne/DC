if exist('./env.mat','file')==2 
    delete 'env.mat'; % in case it got deposited by accident
end
cd Obstacle/
MakeParameters;
cd ..
logging = PlayGame('Obstacle',env);

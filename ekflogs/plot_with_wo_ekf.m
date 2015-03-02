% With EKF
load WithEKF.csv;
withEKF_X = WithEKF(:,1);
withEKF_Y = WithEKF(:,2);


% Without EKF
load WoEKF.csv;    
woEKF_X = WoEKF(:,1);   
woEKF_Y = WoEKF(:,2);   

% Actual
load Actual.csv;    
Actual_X = Actual(:,1);   
Actual_Y = Actual(:,2);

figure
plot(Actual_X,Actual_Y, '-xk', withEKF_X,withEKF_Y, ':ob',woEKF_X,woEKF_Y, '--*r' ), legend('Actual', 'With EKF', 'Without EKF', 'Location','northwest');

xlabel('x');           % add axis labels and plot title
ylabel('y');
axis equal;
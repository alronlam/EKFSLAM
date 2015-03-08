% Breadcrumb: INS + VINS Dummies
%load breadcrumb.csv;
%breadcrumb_X = breadcrumb(:,1);
%breadcrumb_Y = breadcrumb(:,2);

% Breadcrumb: INS (with Cimu heading) + VINS Dummies
load breadcrumbCimuHeading.csv;
breadcrumbCimuHeading_X = breadcrumbCimuHeading(:,1);
breadcrumbCimuHeading_Y = breadcrumbCimuHeading(:,2);

% Breadcrumb 15Hz: INS (with Cimu heading) + VINS Dummies
load breadcrumbCimuHeading15hz.csv;
breadcrumbCimuHeading15hz_X = breadcrumbCimuHeading15hz(:,1);
breadcrumbCimuHeading15hz_Y = breadcrumbCimuHeading15hz(:,2);



% INS
load ins.csv;    
ins_X = ins(:,1);   
ins_Y = ins(:,2);   

% INS With Cimu Heading
load insCimuHeading.csv;    
insCimuHeading_X = insCimuHeading(:,1);   
insCimuHeading_Y = insCimuHeading(:,2);   

% Double Integration
load doubleintegration.csv;    
doubleintegration_X = doubleintegration(:,1);   
doubleintegration_Y = doubleintegration(:,2);

% VINS
load vins.csv;    
vins_X = vins(:,1);   
vins_Y = vins(:,2);

% VINS - 15Hz
load vins15hz.csv;    
vins15hz_X = vins15hz(:,1);   
vins15hz_Y = vins15hz(:,2);

% VINS IDP
%load vinsidp.csv;    
%vinsidp_X = vinsidp(:,1);   
%vinsidp_Y = vinsidp(:,2);

%Miguel 4th Ground Truth
miguel4_x = [0.0, -24.867919469670156, -21.367919469670156, 3.4999999999999996, 0.0];
miguel4_y = [0.0, -14.357499999999998, -20.419677826491068, -6.062177826491071, 0.0];

%Yuch Lobby Ground Truth
yuch_x = [-0.0, 12.199999999999994, 5.531604390859815, -6.668395609140179, -0.0];
yuch_y = [0.0, -21.1310198523403, -24.9810198523403, -3.8499999999999983, 0.0];

% Yuch Smaller Rectangle Ground Truth
yuchsmall_x = [0.0, -9.646890864181575, -7.068065416152368, 2.5788254480292063, 0.0];
yuchsmall_y = [0.0, -4.094862226567589, -10.170194259353694, -6.075332032786106, 0.0];

% SJ Ground Truth
sj_x = [0.0, -40.317769744888714];
sj_y = [0.0, 90.55519348882281];

% SJ Partial Ground Truth
sjpart_x = [0.0, -32.626379824325326];
sjpart_y = [0.0, 73.28004888480123];

% LS Ground Truth
ls_x = [0.0, 45.7619397124583];
ls_y = [0.0, -102.78299943936902];

% LS Reverse Ground Truth
lsrev_x = [-0.0, -45.76193971245827];
lsrev_y = [0.0, 102.78299943936904];

% 0 to plot everything, 1 to plot only ins, vins, and breadcrumb
plotMode = -1;

figure
hold on

% Ground Truth

% Miguel 4th
%plot(miguel4_x, miguel4_y, 'k', 'LineWidth', 5, 'DisplayName', 'Miguel 4th Ground Truth');

% Yuch Lobby
%plot(yuch_x, yuch_y, 'k', 'LineWidth', 5, 'DisplayName', 'Yuch Lobby Ground Truth');

% Yuch Smaller Rectangle
%plot(yuchsmall_x, yuchsmall_y, 'k', 'LineWidth', 5, 'DisplayName', 'Yuch Smaller Rectangle Ground Truth');

% SJ
%plot(sj_x, sj_y, 'k', 'LineWidth', 5, 'DisplayName', 'SJ Ground Truth');

% SJ
%plot(sjpart_x, sjpart_y, 'k', 'LineWidth', 5, 'DisplayName', 'SJ Partial Ground Truth');

% LS
%plot(ls_x, ls_y, 'k', 'LineWidth', 5, 'DisplayName', 'LS Ground Truth');

% LS Reverse
plot(lsrev_x, lsrev_y, 'k', 'LineWidth', 5, 'DisplayName', 'LS Reverse Ground Truth');


if(plotMode == 0)
    %Double Integration
    plot(doubleintegration_X,doubleintegration_Y, '-xm', 'DisplayName', 'Double Integration');

    %V-INS 15Hz
    plot(vins15hz_X,vins15hz_Y, ':xg', 'DisplayName', 'V-INS 15Hz');
    
    %Breadcrumb with Cimu Heading 15Hz
    plot(breadcrumbCimuHeading15hz_X,breadcrumbCimuHeading15hz_Y, ':ok', 'DisplayName', 'Breadcrumb 15Hz w/ Cimu Heading');

end
if(plotMode ~= -1)
    %V-INS
    plot(vins_X,vins_Y, ':xr', 'DisplayName', 'V-INS');

    %INS
    plot(ins_X,ins_Y, '--xb', 'DisplayName', 'INS');

    %INS with Cimu Heading
    %plot(insCimuHeading_X,insCimuHeading_Y, '--xc', 'DisplayName', 'INS w/ Cimu Heading');

    %Breadcrumb with Cimu Heading
    plot(breadcrumbCimuHeading_X,breadcrumbCimuHeading_Y, ':oy', 'DisplayName', 'Breadcrumb w/ Cimu Heading'); hold on;

    %Breadcrumb
    %plot(breadcrumb_X,breadcrumb_Y, ':og', 'DisplayName', 'Breadcrumb');

    %V-INS IDP
    %plot(vinsidp_X,vinsidp_Y, ':oc', 'DisplayName', 'IDP');
end
legend('-DynamicLegend');



%plot(doubleintegration_X, doubleintegration_Y, '-xg'), legend('Double Integration');
%hold on
%plot(vinsidp_X, vinsidp_Y, '-xk');
%plot(vins_X, vins_Y, '-xr'), legend('V-INS');%, ins_X,ins_Y, ':ob');

%legend (['Double Integration' , 'V-INS']);
%plot(ins_X,ins_Y, ':ob');
%plot(breadcrumb_X,breadcrumb_Y, '-xk', ins_X,ins_Y, ':ob',vins_X,vins_Y, '--*r', vinsidp_X, vinsidp_Y, '--*c'  ), legend('Breadcrumb', 'INS', 'V-INS', 'Pure IDP', 'Location','northwest');

%plot(breadcrumb_X,breadcrumb_Y, '-xk', ins_X,ins_Y, ':ob',vins_X,vins_Y, '--*r' ), legend('Breadcrumb', 'INS', 'V-INS', 'Location','northwest');

xlabel('x');           % add axis labels and plot title
ylabel('y');
axis equal;
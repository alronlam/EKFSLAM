% INS + VINS Dummies
load breadcrumb.csv;
breadcrumb_X = breadcrumb(:,1);
breadcrumb_Y = breadcrumb(:,2);

% INS
load ins.csv;    
ins_X = ins(:,1);   
ins_Y = ins(:,2);   

% INS With Cimu Heading
load insCimuHeading.csv;    
insCimuHeading_X = insCimuHeading(:,1);   
insCimuHeading_Y = insCimuHeading(:,2);   


% VINS
load vins.csv;    
vins_X = vins(:,1);   
vins_Y = vins(:,2);

% VINS IDP
load vinsidp.csv;    
vinsidp_X = vinsidp(:,1);   
vinsidp_Y = vinsidp(:,2);

% Double Integration
load doubleintegration.csv;    
doubleintegration_X = doubleintegration(:,1);   
doubleintegration_Y = doubleintegration(:,2);


figure
hold on

%Double Integration
plot(doubleintegration_X,doubleintegration_Y, '-xm', 'DisplayName', 'Double Integration');

%V-INS
plot(vins_X,vins_Y, ':xr', 'DisplayName', 'V-INS');

%INS
plot(ins_X,ins_Y, ':xb', 'DisplayName', 'INS');

%INS with Cimu Heading
plot(insCimuHeading_X,insCimuHeading_Y, ':xc', 'DisplayName', 'INS w/ Cimu Heading');

%Breadcrumb
%plot(breadcrumb_X,breadcrumb_Y, ':og', 'DisplayName', 'Breadcrumb');

%V-INS IDP
%plot(vinsidp_X,vinsidp_Y, ':oc', 'DisplayName', 'IDP');

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
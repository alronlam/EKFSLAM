% Breadcrumb: INS + VINS Dummies
load breadcrumb.csv;
breadcrumb_X = breadcrumb(:,1);
breadcrumb_Y = breadcrumb(:,2);

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
load vinsidp.csv;    
vinsidp_X = vinsidp(:,1);   
vinsidp_Y = vinsidp(:,2);



figure
hold on

%Double Integration
plot(doubleintegration_X,doubleintegration_Y, '-xm', 'DisplayName', 'Double Integration');

%V-INS
plot(vins_X,vins_Y, ':xr', 'DisplayName', 'V-INS'); hold on;

%V-INS 15Hz
plot(vins15hz_X,vins15hz_Y, ':xg', 'DisplayName', 'V-INS 15Hz');

%INS
%plot(ins_X,ins_Y, ':xb', 'DisplayName', 'INS');

%INS with Cimu Heading
%plot(insCimuHeading_X,insCimuHeading_Y, ':xc', 'DisplayName', 'INS w/ Cimu Heading');

%Breadcrumb
%plot(breadcrumb_X,breadcrumb_Y, ':og', 'DisplayName', 'Breadcrumb');

%Breadcrumb with Cimu Heading
plot(breadcrumbCimuHeading_X,breadcrumbCimuHeading_Y, ':oy', 'DisplayName', 'Breadcrumb w/ Cimu Heading'); hold on;

%Breadcrumb with Cimu Heading 15Hz
plot(breadcrumbCimuHeading15hz_X,breadcrumbCimuHeading15hz_Y, ':ok', 'DisplayName', 'Breadcrumb 15Hz w/ Cimu Heading');


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
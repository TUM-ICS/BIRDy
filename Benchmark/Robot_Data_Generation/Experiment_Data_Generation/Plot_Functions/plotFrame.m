function [Frame]=plotFrame(HT, name, options)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Given an absolute homogene transformation plots the coordinate frame
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if nargin < 3
    options.displayText = true;
end

Frame.name = name;
tOff = 0.01;
k=0.02; %coef to change vectors length

R = HT(1:3,1:3);
t = HT(1:3,4);

axisX=k*R(:,1)+t;
axisY=k*R(:,2)+t;
axisZ=k*R(:,3)+t;


% Origin O of the frame
Frame.O = plot3(t(1),t(2),t(3), 'k .','MarkerSize',20,'HandleVisibility','off');
if options.displayText == true
    Frame.nameTag = text(t(1)+tOff,t(2)+tOff,t(3)+tOff, Frame.name);
end
hold on
%Plot x-axis
Frame.X = plot3([t(1);axisX(1)],[t(2);axisX(2)],[t(3);axisX(3)],'r -', 'LineWidth', 4,'HandleVisibility','off');
hold on
%Plot y-axis
Frame.Y = plot3([t(1);axisY(1)],[t(2);axisY(2)],[t(3);axisY(3)],'g -', 'LineWidth', 4,'HandleVisibility','off');
hold on
%Plot z-axis
Frame.Z = plot3([t(1);axisZ(1)],[t(2);axisZ(2)],[t(3);axisZ(3)],'b -', 'LineWidth', 4,'HandleVisibility','off');

end


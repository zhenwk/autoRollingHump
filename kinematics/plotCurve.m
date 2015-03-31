function [  ] = plotCurve( CurveInfo )
%
% plot3(CurveInfo.r.x(1:CurveInfo.EndIndex),CurveInfo.r.y(1:CurveInfo.EndIndex),...
%         CurveInfo.r.z(1:CurveInfo.EndIndex),'LineWidth',1.5);
plot(CurveInfo.r.z(1:CurveInfo.EndIndex),'LineWidth',1.5);

xlabel('x');
ylabel('y');
zlabel('z');
grid on;
% xlim([-0.2,0.2]);
% ylim([-0.2,0.2]);
% zlim([0,0.17]);
% axis equal


end


function [ newAngles ] = formatAngle( oldAngles )
%FORMATANGLE format angles such that it is always +-[0 2*pi]
newAngles= zeros(length(oldAngles));
for i=1:length(oldAngles)
    newAngles(i)=mod(oldAngles(i),360);
end
end


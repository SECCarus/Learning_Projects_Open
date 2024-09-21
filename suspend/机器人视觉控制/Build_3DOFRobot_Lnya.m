% %Build Robot by D_H methods

global L1 L2 L3 L4 L5;

ToDeg = 180/pi;
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';

% 这里修改机器人的关节信息
Link= struct('name','Body' , 'th',  0, 'dz', 0, 'dx', 0, 'alf',90*ToRad,'az',UZ);     % az 
Link(1)= struct('name','Base' , 'th',  -180*ToRad, 'dz', 0, 'dx', 0, 'alf',0*ToRad,'az',UZ);        %Base To 1
Link(2) = struct('name','J1' , 'th',   0*ToRad, 'dz', L1, 'dx', 0, 'alf',90*ToRad,'az',UZ);       %1 TO 2
Link(3) = struct('name','J2' , 'th',  0*ToRad, 'dz', 0, 'dx', L2, 'alf',0*ToRad,'az',UZ);    %2 TO 3
Link(4) = struct('name','J3' , 'th',  0*ToRad, 'dz', 0, 'dx', L3, 'alf',0*ToRad,'az',UZ);          %3 TO E
Link(5) = struct('name','J4' , 'th',  0*ToRad, 'dz', 0, 'dx', L4, 'alf',0*ToRad,'az',UZ);
Link(6) = struct('name','J5' , 'th',  90*ToRad, 'dz', 0, 'dx', L5, 'alf',-90*ToRad,'az',UX);
Link(7) = struct('name','tool' , 'th',  0*ToRad, 'dz', 0, 'dx', 20, 'alf',0*ToRad,'az',UX);

                           
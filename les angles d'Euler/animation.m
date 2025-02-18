function animation(positions,angles)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
l= 1;
axle_x = [-l/2 0 0;
           l/2 0 0;
           -l/2*cos(pi/3) -l/2*sin(pi/3) 0;
           l/2*cos(pi/3) l/2*sin(pi/3) 0;
           -l/2*cos(pi/3) l/2*sin(pi/3) 0;
           l/2*cos(pi/3) -l/2*sin(pi/3) 0];
      

r = 0.1*l; 
ang = linspace(0,2*pi);
x_circle = r*cos(ang);
y_circle = r*sin(ang);
z_circle = zeros(1,length(ang));
propeller = [x_circle',y_circle',z_circle'];


[p1,q1] = size(propeller);
[p2,q2] = size(axle_x);
[mm,nn] = size(angles);

for ii=1:mm
    x = positions(ii,1);
    y = positions(ii,2);
    z = positions(ii,3);
    phi = angles(ii,1); 
    theta = angles(ii,2);
    psi = angles(ii,3);
    R = get_rotation(phi,theta,psi);
    
    for i=1:p2
        r_body = axle_x(i,:)';
        r_world = R*r_body;
        new_axle_x(i,:) = r_world';
    end
    new_axle_x = [x y z] +new_axle_x;
    
    
    
    for i=1:p1
        r_body = propeller(i,:)';
        r_world = R*r_body;
        new_propeller(i,:) = r_world'; 
    end
    new_propeller1 = new_axle_x(1, :) + new_propeller;
    new_propeller2 = new_axle_x(2, :) + new_propeller;
    new_propeller3 = new_axle_x(3, :) + new_propeller;
    new_propeller4 = new_axle_x(4, :) + new_propeller;
    new_propeller5 = new_axle_x(5, :) + new_propeller;
    new_propeller6 = new_axle_x(6, :) + new_propeller;

    line(new_axle_x(1:2, 1), new_axle_x(1:2, 2), new_axle_x(1:2, 3),'Color', 'black', 'Linewidth', 2); hold on;
    line(new_axle_x(3:4, 1), new_axle_x(3:4, 2), new_axle_x(3:4, 3),'Color', 'black', 'Linewidth', 2); hold on;
    line(new_axle_x(5:6, 1), new_axle_x(5:6, 2), new_axle_x(5:6, 3),'Color', 'black', 'Linewidth', 2); hold on;
    patch(new_propeller1(:, 1), new_propeller1(:, 2), new_propeller1(:, 3), 'r');
    patch(new_propeller2(:, 1), new_propeller2(:, 2), new_propeller2(:, 3), 'g');
    patch(new_propeller3(:, 1), new_propeller3(:, 2), new_propeller3(:, 3), 'b');
    patch(new_propeller4(:, 1), new_propeller4(:, 2), new_propeller4(:, 3), 'c');
    patch(new_propeller5(:, 1), new_propeller5(:, 2), new_propeller5(:, 3), 'm');
    patch(new_propeller6(:, 1), new_propeller6(:, 2), new_propeller6(:, 3), 'y');
    axis(1 * [-2.5 2.5 -2.5 2.5 -2.5 2.5]);
    xlabel('x'); ylabel('y'); zlabel('z');
    view(3)
    pause(0.01)
    if (ii ~= mm)
        clf
    end
end
end
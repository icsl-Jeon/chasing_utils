function h = draw_ellipse(R,u,c,C,ec)
% draw_ellipse(R,u,c,C,ec) 
% R = SO(2) / u = [u1 u2] /  c = [cx cy] / C = color / ec = edge color
c = c(:);
ang=0:0.01:2*pi; 
Xs = [cos(ang) ; sin(ang)];
Xs = R*diag(u)*Xs+c;

h = fill(Xs(1,:),Xs(2,:),C,'EdgeColor',ec);

end
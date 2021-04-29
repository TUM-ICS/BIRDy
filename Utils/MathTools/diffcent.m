function yd = diffcent(y,pas)
%DIFFCENT	derivee numerique par difference centrale
%	DIFFCENT(Y, PAS), for a vector Y, is [Y(2)-Y(1) (Y(3)-Y(2))/2 ... Y(n)-Y(n-1)]/PAS.
%
%	See also DIFF, GRADIENT, DEL2, INT, SYMVAR.

%	Copyright (c) 1994 by M. Gautier, LAN Robotique
%	Exemple : yd=diffcent(y,pas);

ny=length(y);
yd=[(y(2)-y(1)),(y(3:ny)-y(1:ny-2))/2,(y(ny)-y(ny-1))]/pas;
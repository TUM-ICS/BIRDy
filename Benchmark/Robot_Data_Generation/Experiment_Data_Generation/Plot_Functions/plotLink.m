function plotLink(link)

%% Plot a .STL robot segment

patch(link,'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'none', 'AmbientStrength', 0.5,'FaceAlpha',0.5,'HandleVisibility','off');
material('metal');
camlight('headlight', 'infinite');

end

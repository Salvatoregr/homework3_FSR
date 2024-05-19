function h = personal_plot1(x, y, x_label_latex, y_label_latex, pdf_name)

set(0, 'DefaultTextInterpreter', 'latex')
set(0, 'DefaultLegendInterpreter', 'latex')
set(0, 'DefaultAxesTickLabelInterpreter', 'latex')
lw = 2;

h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);
removeToolbarExplorationButtons(h)

plot(x, y, 'Linewidth', lw);
legend('u_T');
xlim([x(1) x(end)])

xlabel(x_label_latex)
ylabel(y_label_latex)
set(gca, 'FontSize',18);
grid on
box on
set(gcf,'color','w');
exportgraphics(h, pdf_name);

end

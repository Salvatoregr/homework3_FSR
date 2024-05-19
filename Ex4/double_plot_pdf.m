function h = personal_plot2(x, y1, y2, x_label_latex, y_label_latex, y1_legend_latex, y2_legend_latex, pdf_name)

set(0, 'DefaultTextInterpreter', 'latex')
set(0, 'DefaultLegendInterpreter', 'latex')
set(0, 'DefaultAxesTickLabelInterpreter', 'latex')
lw = 2;

h = figure('Renderer', 'painters', 'Position', [10 10 900 350]);
removeToolbarExplorationButtons(h)

plot(x, y1, 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);
hold on
plot(x, y2, 'k--', 'Linewidth', lw ,'Color', [0.5, 0.5, 0.5]);



legend(y1_legend_latex, y2_legend_latex);
xlabel(x_label_latex)
ylabel(y_label_latex)
xlim([x(1) x(end)])
set(gca, 'FontSize',18);
grid on
box on
set(gcf,'color','w');
legend('Location','northoutside','Orientation','horizontal')
exportgraphics(h, pdf_name);

end

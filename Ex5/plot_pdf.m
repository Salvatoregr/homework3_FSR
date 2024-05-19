function h = plot_pdf(x, y1, y2,y3, x_label_latex, y_label_latex, y1_legend_latex, y2_legend_latex,y3_legend_latex, pdf_name)

set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');
lw = 2;

h = figure();
removeToolbarExplorationButtons(h);

plot(x, y1, 'r-', 'Linewidth', lw);
hold on
plot(x, y2, 'g-', 'Linewidth', lw);
hold on
plot(x, y3, 'b-', 'Linewidth', lw);


legend(y1_legend_latex, y2_legend_latex,y3_legend_latex);
xlabel(x_label_latex);
ylabel(y_label_latex);
xlim([x(1) x(end)]);
set(gca, 'FontSize',18);
grid on
box on
set(gcf,'color','w');
%legend('Location','northoutside','Orientation','horizontal')
exportgraphics(h, pdf_name);

end

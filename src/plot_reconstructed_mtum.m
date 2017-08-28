function plot_reconstructed_mtum(act_sol, comp_sol, reg_mat, out_vec, is_planar)
    mtum_act = reg_mat * act_sol.' - out_vec;
    mtum_comp = reg_mat * comp_sol.' - out_vec;
    figure();
    if is_planar
        subplot(1, 3, 1)
        plot(mtum_act(1 : 3 : end)); hold on;
        plot(mtum_comp(1 : 3 : end)); legend('P_{xa}','P_{xc}');
        subplot(1, 3, 2)
        plot(mtum_act(2 : 3 : end)); hold on;
        plot(mtum_comp(2 : 3 : end)); legend('P_{ya}', 'P_{yc}');
        subplot(1, 3, 3)
        plot(mtum_act(3 : 3 : end)); hold on;
        plot(mtum_comp(3 : 3 : end)); legend('L_{za}', 'L_{zc}');
    else
        subplot(2, 3, 1)
        plot(mtum_act(1 : 6 : end)); hold on;
        plot(mtum_comp(1 : 6 : end)); legend('P_{xa}','P_{xc}');
        subplot(2, 3, 2)
        plot(mtum_act(2 : 6 : end)); hold on;
        plot(mtum_comp(2 : 6 : end)); legend('P_{ya}', 'P_{yc}');
        subplot(2, 3, 3)
        plot(mtum_act(3 : 6 : end)); hold on;
        plot(mtum_comp(3 : 6 : end)); legend('P_{za}', 'P_{zc}');
        subplot(2, 3, 4)
        plot(mtum_act(4 : 6 : end)); hold on;
        plot(mtum_comp(4 : 6 : end)); legend('L_{xa}','L_{xc}');
        subplot(2, 3, 5)
        plot(mtum_act(5 : 6 : end)); hold on;
        plot(mtum_comp(5 : 6 : end)); legend('L_{ya}', 'L_{yc}');
        subplot(2, 3, 6)
        plot(mtum_act(6 : 6 : end)); hold on;
        plot(mtum_comp(6 : 6 : end)); legend('L_{za}', 'L_{zc}');
    end        
end


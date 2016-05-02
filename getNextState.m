
function [rhoNext, kappaNext] = getNextState(rho, kappa, h, nu, u)

    g = @(z)...
        [((z(1) - rho)/h) - (-nu*cos((z(2) + kappa)/2)) ; ...
        ((z(2) - kappa)/h) - (-nu*u) - (2/(z(1)+rho))*nu*sin((z(2)+kappa)/2)];
    options = optimset('Display','off');
    fprintf('rho = %f, kappa = %f\n', rho, kappa)
    g([rho, kappa]);
    nextState = fsolve(g, ([rho;kappa]), options);
    rhoNext = nextState(1);
    kappaNext = nextState(2);
end




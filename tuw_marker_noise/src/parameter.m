function p = parameter(file)
  pkg load optim

  % define measurement noise model
  F_par = @(x, p) [x .* x, ones(1,rows(x))'] * p;
  F_par_inv = @(x, p) [(x .* x).^(-1), ones(1,rows(x))'] * p;

  % load sample data
  CSV = dlmread(file, ';', 1, 0);
  assert (columns(CSV) == 9)

  % initialization (see http://octave.sourceforge.net/optim/function/leasqr.html)
  X = CSV(:, 1:3);
  Y = CSV(:, 4:6);
  pin = ones(1, 2);
  stol = 0.0001;
  niter = 50;

  % find optimal parameters for variance r
  [f_tmp, p_r_r]     = leasqr (X(:,1), Y(:,1), [1,1], F_par, stol, niter);
  [f_tmp, p_r_phi]   = leasqr (X(:,2), Y(:,1), [1,1], F_par, stol, niter);
  [f_tmp, p_r_theta] = leasqr (X(:,3), Y(:,1), [1,1], F_par, stol, niter);

  % plot variance r sub-results
  x = 0:0.1:8;
  figure(1);
  plot(X(:,1), Y(:,1), '.', x, max(F_par(x', p_r_r), 0), '-');
  xlabel('r');
  ylabel('\sigma_r^2');
  legend('real','model');
  % axis([0, 8, 0, 2]);

  x = -pi:0.1:pi;
  figure(2);
  plot(X(:,2), Y(:,1), '.', x, max(min(F_par(x', p_r_phi), pi^2), 0), '-');
  xlabel('\phi');
  ylabel('\sigma_r^2');
  legend('real','model');
  % axis([-1, 1, 0, 0.2]);

  x = -pi:0.1:pi;
  figure(3);
  plot(X(:,3), Y(:,1), '.', x, max(min(F_par(x', p_r_theta), pi^2), 0), '-');
  xlabel('\theta');
  ylabel('\sigma_r^2');
  legend('real','model');
  % axis([-2, 2, 0, 0.4]);

  % calculate percent of single variances bigger than global variance for r
  y = max(F_par(X(:,1), p_r_r), 0) + max(min(F_par(X(:,2), p_r_phi), pi^2), 0) + max(min(F_par(X(:,3), p_r_theta), pi^2), 0);
  c = 0;
  for i = 1:length(y)
    if Y(i,1) > y(i)
      c = c + 1;
    endif
  endfor
  c/length(y) * 100

  % find optimal parameters for variance phi
  [f_tmp, p_phi_r]     = leasqr (X(:,1), Y(:,2), [1,1], F_par_inv, stol, niter);
  [f_tmp, p_phi_phi]   = leasqr (X(:,2), Y(:,2), [1,1], F_par, stol, niter);
  [f_tmp, p_phi_theta] = leasqr (X(:,3), Y(:,2), [1,1], F_par, stol, niter);

  % plot variance phi sub-results
  x = 0:0.1:8;
  figure(4);
  plot(X(:,1), Y(:,2), '.', x, max(F_par_inv(x', p_phi_r), 0), '-');
  xlabel('r');
  ylabel('\sigma_\phi^2');
  legend('real','model');
  % axis([0, 8, 0, 0.1]);

  x = -pi:0.1:pi;
  figure(5);
  plot(X(:,2), Y(:,2), '.', x, max(min(F_par(x', p_phi_phi), pi^2), 0), '-');
  xlabel('\phi');
  ylabel('\sigma_\phi^2');
  legend('real','model');
  % axis([-1, 1, 0, 0.04]);

  x = -pi:0.1:pi;
  figure(6);
  plot(X(:,3), Y(:,2), '.', x, max(min(F_par(x', p_phi_theta), pi^2), 0), '-');
  xlabel('\theta');
  ylabel('\sigma_\phi^2');
  legend('real','model');
  #axis([-2, 2, 0, 0.7]);

  % calculate percent of single variances bigger than global variance for phi
  y = min(max(F_par(X(:,1), p_phi_r), 0) + max(min(F_par(X(:,2), p_phi_phi), pi^2), 0) + max(min(F_par(X(:,3), p_phi_theta), pi^2), 0), pi^2);
  c = 0;
  for i = 1:length(y)
    if Y(i,2) > y(i)
      c = c + 1;
    endif
  endfor
  c/length(y) * 100

  % find optimal parameters for theta variance
  [f_tmp, p_theta_r]     = leasqr (X(:,1), Y(:,3), [1,1], F_par, stol, niter);
  [f_tmp, p_theta_phi]   = leasqr (X(:,2), Y(:,3), [1,1], F_par, stol, niter);
  [f_tmp, p_theta_theta] = leasqr (X(:,3), Y(:,3), [1,1], F_par, stol, niter);

  % plot variance theta sub-results
  x = 0:0.1:8;
  figure(7);
  plot(X(:,1), Y(:,3), '.', x, max(F_par(x', p_theta_r), 0), '-');
  xlabel('r');
  ylabel('\sigma_\theta^2');
  legend('real','model');
  % axis([0, 8, 0, 2]);

  x = -pi:0.1:pi;
  figure(8);
  plot(X(:,2), Y(:,3), '.', x, max(min(F_par(x', p_theta_phi), pi^2), 0), '-');
  xlabel('\phi');
  ylabel('\sigma_\theta^2');
  legend('real','model');
  % axis([-0.8, 0.8, 0, 6]);

  x = -pi:0.1:pi;
  figure(9);
  plot(X(:,3), Y(:,3), '.', x, max(min(F_par(x', p_theta_theta), pi^2), 0), '-');
  xlabel('\theta');
  ylabel('\sigma_\theta^2');
  legend('real','model');
  % axis([-2, 2, 0, 6]);

  % calculate percent of single variances bigger than global variance for theta
  y = min(max(F_par(X(:,1), p_theta_r), 0) + max(min(F_par(X(:,2), p_theta_phi), pi^2), 0) + max(min(F_par(X(:,3), p_theta_theta), pi^2), 0), pi^2);
  c = 0;
  for i = 1:length(y)
    if Y(i,3) > y(i)
      c = c + 1;
    endif
  endfor
  c/length(y) * 100

  p = [p_r_r', p_r_phi', p_r_theta'; p_phi_r', p_phi_phi', p_phi_theta'; p_theta_r', p_theta_phi', p_theta_theta'];
endfunction

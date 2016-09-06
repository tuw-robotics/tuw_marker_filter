function p = compare(recordfile, parameterfile)
  % define measurement noise model
  F_par = @(x, p) [x .* x, ones(1,rows(x))'] * p;
  F_par_inv = @(x, p) [(x .* x).^(-1), ones(1,rows(x))'] * p;

  % define angle difference
  adiff = @(a,b) atan2(sin(a-b), cos(a-b));

  % load sample data
  R = dlmread(recordfile, ';', 1, 0);
  assert (columns(R) == 6)
  X = R(:, 1:3);
  Z = R(:, 4:6);

  % load parameter data
  P = dlmread(parameterfile, ';', 1, 1);
  assert (rows(P) == 3 && columns(P) == 6)

  % calculate difference between expectation and measurement
  D = abs(X - Z);
  D(:,2) = abs(adiff(X(:,2), Z(:,2)));
  D(:,3) = abs(adiff(X(:,3), Z(:,3)));

  % plot variance r sub-results
  x = 0:0.1:8;
  figure(1);
  plot(X(:,1), D(:,1), '.', x, sqrt(max(F_par(x', [P(1,1); P(1,2)]), 0)), '-');
  xlabel('\mu_r');
  ylabel('d_r, \sigma_r');
  legend('real','model');

  x = -pi:0.1:pi;
  figure(2);
  plot(X(:,2), D(:,1), '.', x, sqrt(max(min(F_par(x', [P(1,3); P(1,4)]), pi^2), 0)), '-');
  xlabel('\mu_\phi');
  ylabel('d_r, \sigma_r');
  legend('real','model');

  x = -pi:0.1:pi;
  figure(3);
  plot(X(:,3), D(:,1), '.', x, sqrt(max(min(F_par(x', [P(1,5); P(1,6)]), pi^2), 0)), '-');
  xlabel('\mu_\theta');
  ylabel('d_r, \sigma_r');
  legend('real','model');

  % plot variance phi sub-results
  x = 0:0.01:8;
  figure(4);
  plot(X(:,1), D(:,2), '.', x, sqrt(max(F_par_inv(x', [P(2,1); P(2,2)]),0)), '-');
  xlabel('\mu_r');
  ylabel('d_\phi, \sigma_\phi');
  legend('real','model');

  x = -pi:0.1:pi;
  figure(5);
  plot(X(:,2), D(:,2), '.', x, sqrt(max(min(F_par(x', [P(2,3); P(2,4)]), pi^2), 0)), '-');
  xlabel('\mu_\phi');
  ylabel('d_\phi, \sigma_\phi');
  legend('real','model');

  x = -pi:0.1:pi;
  figure(6);
  plot(X(:,3), D(:,2), '.', x, sqrt(max(min(F_par(x', [P(2,5); P(2,6)]), pi^2), 0)), '-');
  xlabel('\mu_\theta');
  ylabel('d_\phi, \sigma_\phi');
  legend('real','model');

  % plot variance theta sub-results
  x = 0:0.1:8;
  figure(7);
  plot(X(:,1), D(:,3), '.', x, sqrt(max(F_par(x', [P(3,1); P(3,2)]), 0)), '-');
  xlabel('\mu_r');
  ylabel('d_\theta, \sigma_\theta');
  legend('real','model');

  x = -pi:0.1:pi;
  figure(8);
  plot(X(:,2), D(:,3), '.', x, sqrt(max(min(F_par(x', [P(3,3); P(3,4)]), pi^2), 0)), '-');
  xlabel('\mu_\phi');
  ylabel('d_\theta, \sigma_\theta');
  legend('real','model');

  x = -pi:0.1:pi;
  figure(9);
  plot(X(:,3), D(:,3), '.', x, sqrt(max(min(F_par(x', [P(3,5); P(3,6)]), pi^2), 0)), '-');
  xlabel('\mu_\theta');
  ylabel('d_\theta, \sigma_\theta');
  legend('real','model');
endfunction

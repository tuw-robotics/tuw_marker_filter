function p = parameter(file)
  pkg load optim

  % define measurement noise model
  F = @(x, p) (x .* x) * [p(1), p(2); p(3), p(4); p(5), p(6)]';

  % load sample data
  CSV = dlmread(file, ';', 1, 0);
  assert (columns(CSV) == 6)

  % initialization (see http://octave.sourceforge.net/optim/function/leasqr.html)
  X = CSV(:, 1:2);
  Y = CSV(:, 4:6);
  pin = ones(1, 9);
  stol = 0.0001;
  niter = 50;
  wt = 1 ./ sqrt(Y);
  dp = 0.001 * ones (size (pin));

  % find optimal parameters
  [f_tmp, p_tmp] = leasqr (X, Y, pin, F, stol, niter, wt, dp);

  p = [p_tmp(1), p_tmp(2); p_tmp(3), p_tmp(4); p_tmp(5), p_tmp(6)];
endfunction

function [amplitude, w] = get_MRFT_a_and_w(transfer_function, h_mrft, beta_mrft)
%Returns the estimated amplitude and frequency of mrft oscilations

p = rad2deg(asin(beta_mrft))-180;

w=logspace(-2,4,10000);
[mag_bode,phase,wout] = bode(transfer_function,w);

amplitude = interp1( squeeze(phase), squeeze(mag_bode), p);
w   = interp1( squeeze(phase), wout, p);

if isnan(w)
    w=logspace(-3,5,10000);
    [mag_bode,phase,wout] = bode(g_model,w);

    amplitude = interp1( squeeze(phase), squeeze(mag_bode), p);
    w   = interp1( squeeze(phase), wout, p);
end
end


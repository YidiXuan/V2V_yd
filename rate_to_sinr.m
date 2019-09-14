function y=rate_to_sinr(rate, band_width)
% band_width_of_cell=180000   3.3734  sinr=9.3633
% band_width_of_mmwave = 2.16 * 10^9    
spectrum_e = rate/band_width;
sinr = 2^(spectrum_e) - 1;
y=sinr;
end
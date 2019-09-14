import matlab.engine

def get_min_rate_of_cell(delay_max, average_length, arrival_rate, min_cut_probably):
    eng = matlab.engine.start_matlab('MATLAB_R2017b')



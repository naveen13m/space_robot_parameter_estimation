function [th, dth, ddth] = cycloidal_traj(thi, thf, t, Tp)
    th = thi + ((thf - thi) / Tp) * (t - (Tp / (2 * pi)) * sin(2 * pi * t / Tp));
    dth = ((thf - thi) / Tp) * (1 - cos(2 * pi * t / Tp));
    ddth = (2 * pi * (thf - thi) / Tp^2) * sin(2 * pi * t / Tp);
end
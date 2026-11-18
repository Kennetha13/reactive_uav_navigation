#ifndef VELOCITY_FUNCTION_H
#define VELOCITY_FUNCTION_H

class VelocityFunction {
    public:
        // Vel(v), Fig. 21
        // HeadingFunction::computeYawHeading will need to be calculated first
        // and fed into #computeVelocity as head_psi
        // vel_max is the largest forward speed a candidate can have (the Vs limit)
        double computeVelocity(
            double candidate_vx,
            double vel_max,
            double k_z,
            double k_psi,
            double head_psi) const;
};
#endif

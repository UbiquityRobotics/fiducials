#include <fiducial_slam/transform_with_variance.h>
#include <cmath>

/*
 * Takes 2 variances and gives the kalman gain figure
 * Kalmain gain represents how much to trust the new measurement compared to the existing one
 * Taken from equation 12 https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
 */
static inline double kalman_gain(double var1, double var2) { return var1 / (var1 + var2); }

/*
 * Returns the probabilty density at a point given the mean and variance of the distribution
 */
static double probabiltyAtPoint(const double x, const double u, const double var) {
    return (1.0 / (std::sqrt(var) * std::sqrt(2.0 * M_PI))) *
           std::exp(-((x - u) * (x - u)) / (2.0 * var));
}

/*
 * Takes in 2 estimates and the combined estimate and calculates the
 * variance of the new estimate to normalize it
 */
static double normalizeDavid(const double newMean, const double mean1, double var1,
                             const double mean2, double var2) {
    // Find the probabilities of the new mean in both original gaussians
    double prob1_at_newMean = probabiltyAtPoint(newMean, mean1, var1);
    double prob2_at_newMean = probabiltyAtPoint(newMean, mean2, var2);
    // We use the sum in quadrature of these 2 probabilities
    double prob_at_newMean = sqrt(pow(prob1_at_newMean, 2) + pow(prob2_at_newMean, 2));

    double newVar = std::pow(1.0 / (prob_at_newMean * sqrt(2.0 * M_PI)), 2);

    // Bound the variance to prevent blow up
    newVar = std::min(newVar, 1e3);
    newVar = std::max(newVar, 1e-8);

    return newVar;
}

// Update this transform with a new one, with variances as weights
// combine variances using David method
// If adding a systematic error is desired it should already be added in
void TransformWithVariance::update(const TransformWithVariance& newT) {
    tf2::Vector3 p1 = transform.getOrigin();
    tf2::Quaternion q1 = transform.getRotation();
    double var1 = variance;

    tf2::Vector3 p2 = newT.transform.getOrigin();
    tf2::Quaternion q2 = newT.transform.getRotation();
    double var2 = newT.variance;

    // Calculate new mean for the position
    // Use equation 15 in article
    double k = kalman_gain(var1, var2);
    transform.setOrigin(p1 + k * (p2 - p1));

    // Calculate new mean for the orientation
    // Use equation 15 in article
    // https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
    //
    // The kalman gain should give us the weight for how far towards the new estimate to go
    // Slerp should put us in a linear frame so the kalman gain should work as is
    // Kalman gain is always [0,1] ?
    transform.setRotation(q1.slerp(q2, k).normalized());

    //
    // Do the new variance calculations
    //

    // Do everything in a 1d space between p1 and p2
    // This should probably use proper multivariate modeling
    double mean1 = 0.0;
    double mean2 = (p2 - p1).length();
    double mean = (transform.getOrigin() - p1).length();

    // Normalize the variances so that the area under the probabilty remains 1
    variance = normalizeDavid(mean, mean1, var1, mean2, var2);
}

// Weighted average of 2 transforms, variances computed using Alexey Method
TransformWithVariance averageTransforms(const TransformWithVariance& t1,
                                        const TransformWithVariance& t2) {
    TransformWithVariance out = t1;
    out.update(t2);
    return out;

    /*
        tf2::Vector3 p1 = t1.transform.getOrigin();
        tf2::Quaternion o1 = t1.transform.getRotation();
        double var1 = t1.variance;

        tf2::Vector3 p2 = t2.transform.getOrigin();
        tf2::Quaternion o2 = t2.transform.getRotation();
        double var2 = t2.variance;

        double k = kalman_gain(var1, var2);

        out.transform.setOrigin(p1 + k * (p2 - p1));
        out.transform.setRotation(q1.slerp(q2, k).normalized());
        out.variance = suminquadrature(var1, var2);

        return out;
    */
}

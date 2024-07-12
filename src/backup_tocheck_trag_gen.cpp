#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include "util.cpp"

using namespace GeographicLib;
using namespace std;

struct SetPoint {
    GPSPosition position_start;
    GPSPosition position_end;
    geometry_msgs::Vector3 x_s;
    geometry_msgs::Vector3 x_f;
    geometry_msgs::Vector3 position;
    geometry_msgs::Vector3 velocity;      // in meters/s
    geometry_msgs::Vector3 acceleration;  // in meters/s
    double yaw;                           // in rad
    double t;
};

GPSPosition launch_position = {41.73724768996549, 12.513644919120955, 5};

Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());

geometry_msgs::Vector3 convertToECEF(GPSPosition positionGPS) {
    geometry_msgs::Vector3 position_ECEF;
    LocalCartesian proj(launch_position.latitude, launch_position.longitude, launch_position.altitude, earth);
    proj.Forward(positionGPS.latitude, positionGPS.longitude, positionGPS.altitude, position_ECEF.x, position_ECEF.y, position_ECEF.z);
    return position_ECEF;
}

geometry_msgs::Vector3 convertToNER(GPSPosition positionGPS, geometry_msgs::Vector3 position_origin = convertToECEF(launch_position)) {
    geometry_msgs::Vector3 position_NED, position_ECEF;
    position_ECEF = convertToECEF(positionGPS);
    position_NED.x = position_ECEF.x - position_origin.x;
    position_NED.y = position_ECEF.y - position_origin.y;
    position_NED.z = position_ECEF.z - position_origin.z;
    return position_NED;
}

struct CubicCoefficients {
    double a0;  // constant term
    double a1;  // coefficient of t
    double a2;  // coefficient of t^2
    double a3;  // coefficient of t^3
};

// Function to calculate cubic polynomial coefficients
CubicCoefficients calculateCubicCoefficients(double r0, double rf) {
    CubicCoefficients coeffs;
    // Boundary conditions
    coeffs.a0 = r0;
    coeffs.a1 = 0.0;  // Assuming rest-to-rest motion
    coeffs.a2 = -3 * (r0 - rf);
    coeffs.a3 = 2 * (r0 - rf);

    return coeffs;
}

// Function to calculate position, velocity, and acceleration at time t
void calculateTrajectory_axis(double r0, double rf, double t, double& position, double& velocity, double& acceleration) {
    CubicCoefficients coeffs = calculateCubicCoefficients(r0, rf);

    position = coeffs.a0 + coeffs.a1 * t + coeffs.a2 * t * t + coeffs.a3 * t * t * t;
    velocity = coeffs.a1 + 2 * coeffs.a2 * t + 3 * coeffs.a3 * t * t;
    acceleration = 2 * coeffs.a2 + 6 * coeffs.a3 * t;
}

// std::vector<GPSPosition> generateTrajectory(const std::vector<GPSPosition>& gps_positions) {
std::vector<SetPoint> generateTrajectory(GPSPosition start, GPSPosition end, double T) {
    std::vector<SetPoint> trajectory;
    const double dt = 0.1;  // time step in seconds

    double t = 0.0;
    // Convert GPS coordinates to local Cartesian coordinates
    geometry_msgs::Vector3 pos_s = convertToNER(start);
    geometry_msgs::Vector3 pos_f = convertToNER(end);

    while (t <= T) {
        SetPoint out;
        out.x_s = pos_s;
        out.x_f = pos_f;
        out.t = t;
        calculateTrajectory_axis(pos_s.x, pos_f.x, t / T, out.position.x, out.velocity.x, out.acceleration.x);
        calculateTrajectory_axis(pos_s.y, pos_f.y, t / T, out.position.y, out.velocity.y, out.acceleration.y);
        calculateTrajectory_axis(pos_s.z, pos_f.z, t / T, out.position.z, out.velocity.z, out.acceleration.z);
        out.yaw = std::atan2(out.velocity.y, out.velocity.x);
        trajectory.push_back(out);
        t += dt;
    }

    return trajectory;
}

int main() {
    // for (size_t i = 0; i < gps_positions.size() - 1; ++i) {
    //     GPSPosition start = gps_positions[i];
    //     GPSPosition end = gps_positions[i + 1];
    GPSPosition start = {41.73720388376838, 12.513649024171759, 5};
    GPSPosition end = {41.73722578686695, 12.513646971647058, 5};

    // // Sample reverse calculation
    // double x = -38e3, y = 230e3, z = -4e3;
    // double lat, lon, h;
    // proj.Reverse(x, y, z, lat, lon, h);
    // cout << lat << " " << lon << " " << h << "\n";
    double duration = 2;
    std::vector<SetPoint> trajectory = generateTrajectory(start, end, duration);
    // for (size_t i = 0; i < trajectory.size(); ++i) {
    //     SetPoint res = trajectory[i];
    //     cout << "i " << i << endl;
    //     cout << "t = " << res.t << endl;
    //     cout << "res s:" << endl;
    //     cout << res.x_s << endl;
    //     cout << "res f:" << endl;
    //     cout << res.x_f << endl;
    //     cout << "res position:" << endl;
    //     cout << res.position << endl;
    //     cout << "res vel:" << endl;
    //     cout << res.velocity << endl;
    //     cout << "res acc:" << endl;
    //     cout << res.acceleration << endl;
    //     cout << "res yaw:" << endl;
    //     cout << res.yaw << endl;
    //     cout << "____________________" << endl;
    // }

    return 0;
}

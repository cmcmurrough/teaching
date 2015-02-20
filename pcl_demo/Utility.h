/***********************************************************************************************************************
 * @FILE Utility.h
 * @BRIEF Header of Utility class
 *
 * This class contains commonly used utility functions and helper classes
 *
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/

#ifndef UTILITY_H
#define UTILITY_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

/***********************************************************************************************************************
 * @CLASS Chronograph
 * @BRIEF Class containing an implementation of a simple timer
 *
 * This class implements a simple timer using boost::chrono
 *
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
class Chronograph
{
public:
    Chronograph()
	{
        reset();
    }
    void reset()
	{
		m_startTime = boost::chrono::steady_clock::now();
		m_isRunning = true;
    }
	void stop()
	{
		m_endTime = boost::chrono::steady_clock::now();
		m_isRunning = false;
    }
	double getTimeMicroseconds()
	{
		boost::chrono::duration<double> duration;
		if(m_isRunning)
		{
			duration = boost::chrono::steady_clock::now() - m_startTime;
		}
		else
		{
			duration = m_endTime - m_startTime;
		}
		boost::chrono::microseconds us = boost::chrono::duration_cast<boost::chrono::microseconds>(duration);
		return static_cast<double>(us.count());
    }
    double getTimeMilliseconds()
	{
		return getTimeMicroseconds() * 0.001;
    }
	double getTimeSeconds()
	{
		return getTimeMicroseconds() * 0.000001;
	}
	static double getMillisecondsSinceEpoch()
	{
		auto now = boost::chrono::steady_clock::now().time_since_epoch();
		auto timeSinceEpoch = boost::chrono::duration_cast<boost::chrono::milliseconds>(now).count();
		return double(timeSinceEpoch);
	}
	static double getSecondsSinceEpoch()
	{
		return getMillisecondsSinceEpoch() * 0.001;
	}
	static std::string getLocalTimeString(const char* format="%y-%m-%d_%H-%M-%S")
	{
		std::stringstream ss;
		boost::posix_time::time_facet *facet = new boost::posix_time::time_facet(format);
		ss.imbue(std::locale(ss.getloc(), facet));
		ss << boost::posix_time::second_clock::local_time();
		return ss.str();
	}
	static void sleep(double ms)
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
private:
	bool m_isRunning;
    boost::chrono::high_resolution_clock::time_point m_startTime;
	boost::chrono::high_resolution_clock::time_point m_endTime;
};

/***********************************************************************************************************************
 * @CLASS Utility
 *
 * @BRIEF Class containing utility functions
 *
 * This class contains several commonly used utility functions. In functions dealing with 3D data, a version optimized
 * for vectorization is provided for speed along with a more general version operating on individual point components.
 * Vector4f is the 3D point structure of choice since Vector3f is not a multiple of 16 bytes (see Eigen documentation)
 *
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
class Utility
{
private:

public:

    // IO
    static void redirectIOToConsole(int maxConsoleLines=500);
	static void printColored(char color, const char* msg, ...);
	static bool writeFile(const char* fileName, std::stringstream &ss);

    // conversions
	static Eigen::Quaternionf rpy2quat(double roll, double pitch, double yaw);
    static void rpy2quat(double roll, double pitch, double yaw, double &qw, double &qx, double &qy, double &qz);
	static Eigen::Quaternionf affine2quat(const Eigen::Affine3f &transformation);
	static Eigen::Affine3f quat2affine(const Eigen::Vector4f &position, const Eigen::Quaternionf &orientation);
	static Eigen::Matrix3f rpy2mat(double roll, double pitch, double yaw);
	static void mat2rpy(const Eigen::Matrix3f &rotation, double &roll, double &pitch, double &yaw);
	static Eigen::Affine3f xyzrpy2affine(const Eigen::Vector4f &position, double roll, double pitch, double yaw);
	static Eigen::Affine3f xyzrpy2affine(double x, double y, double z, double roll, double pitch, double yaw);
	static Eigen::Affine3f basis2affine(const Eigen::Vector4f &position, const Eigen::Vector4f &x, const Eigen::Vector4f &y, const Eigen::Vector4f &z);
	static Eigen::Matrix4f affine2mat(const Eigen::Affine3f &transformation);
	static void cartesian2Spherical(double x, double y, double z, double &radius, double &inclination, double &azimuth);
	static double deg2rad(double degrees);
	static double rad2deg(double radians);
	static Eigen::Vector4f toVec4(Eigen::Vector3f &vec, float lastVal=0); 

	// geometry
    static double pointToLineDistance(const Eigen::Vector4f &p, const Eigen::Vector4f &line_p1, const Eigen::Vector4f &line_p2);
	static double pointToLineDistance(double px, double py, double pz, double line_p1x, double line_p1y, double line_p1z, double line_p2x, double line_p2y, double line_p2z);
	static double pointToRayDistance(const Eigen::Vector4f &p, const Eigen::Vector4f &origin, const Eigen::Vector4f &dir);
	static double pointToRayDistance(double px, double py, double pz, double origin_x, double origin_y, double origin_z, double dir_x, double dir_y, double dir_z);
	static double pointToLineSegmentDistance(const Eigen::Vector4f &p, const Eigen::Vector4f &line_p1, const Eigen::Vector4f &line_p2);
	static double pointToLineSegmentDistance(double px, double py, double pz, double line_p1x, double line_p1y, double line_p1z, double line_p2x, double line_p2y, double line_p2z);
	static double pointToPlaneDistance(const Eigen::Vector4f &p, const Eigen::Vector4f &plane);
	static double pointToPlaneDistance(double px, double py, double pz, double a, double b, double c, double d);
	static double rayToPlaneDistance(const Eigen::Vector4f &origin, const Eigen::Vector4f &direction, const Eigen::Vector4f &plane);
	static double euclideanDistance(const Eigen::Vector4f &p1, const Eigen::Vector4f &p2);
	static double euclideanDistance(double p1x, double p1y, double p1z, double p2x, double p2y, double p2z);
    static void pointAlongLine(const Eigen::Vector4f &line_p1, const Eigen::Vector4f &line_p2, double d, Eigen::Vector4f &p);
    static void pointAlongLine(double line_p1x, double line_p1y, double line_p1z, double line_p2x, double line_p2y, double line_p2z, double d, double &px, double &py, double &pz);
	static void pointAlongRay(const Eigen::Vector4f &origin, const Eigen::Vector4f &direction, double d, Eigen::Vector4f &p);
	static double distanceToTetragon(const Eigen::Vector4f &p, const Eigen::Vector4f &v1, const Eigen::Vector4f &v2, const Eigen::Vector4f &v3, const Eigen::Vector4f &v4);
	static double lineIntersection(const Eigen::Vector4f &line1_p1, const Eigen::Vector4f &line1_p2, const Eigen::Vector4f &line2_p1, const Eigen::Vector4f &line2_p2, Eigen::Vector4f &intersection);
	static bool threePlanesIntersection(const Eigen::Vector4f &plane1, const Eigen::Vector4f &plane2, const Eigen::Vector4f &plane3, Eigen::Vector4f &intersection, double determinantTolerance=0.001);
	static bool linePlaneIntersection(const Eigen::Vector4f &linePoint, const Eigen::Vector4f &lineDir, const Eigen::Vector4f &plane, Eigen::Vector4f &intersection, double paralellTolerance=0.001);
	static void pointsToPlane(const Eigen::Vector4f &p1, const Eigen::Vector4f &p2, const Eigen::Vector4f &p3, Eigen::Vector4f &plane);
	static void pointNormalToPlane(const Eigen::Vector4f &normal, const Eigen::Vector4f &p, Eigen::Vector4f &plane);
	static void orthonormalBasis(const Eigen::Vector4f &u1, const Eigen::Vector4f &u2, const Eigen::Vector4f &u3, Eigen::Vector4f &v1, Eigen::Vector4f &v2, Eigen::Vector4f &v3);
	static bool isPointAbovePlane(const Eigen::Vector4f &p, const Eigen::Vector4f &plane);

	// transforms
	static Eigen::Vector4f transformPoint(const Eigen::Vector4f &p, const Eigen::Matrix4f &transformation);
	static Eigen::Vector3f rotateNormal(const Eigen::Vector3f &v, const Eigen::Matrix3f &rotation);

    // probability
    static double gaussian(double x, double curvePeak, double horizontalOffset, double stdDeviation, double verticalOffset);

    // math
    static bool doubleEquality(double a, double b, int ulp=1);
	static bool doubleAlmostEqual(double a, double b, double maxDifference);
	static bool isPointWithinRange(double x, double y, double z, double minX, double minY, double minZ, double maxX, double maxY, double maxZ);
	static double dotProduct(const Eigen::Vector4f &p1, const Eigen::Vector4f &p2);
	static double dotProduct(double p1x, double p1y, double p1z, double p2x, double p2y, double p2z);
	static Eigen::Vector3f crossProduct(const Eigen::Vector4f &p1, const Eigen::Vector4f &p2);
	static Eigen::Vector3f crossProduct(double p1x, double p1y, double p1z, double p2x, double p2y, double p2z);
	static double squareRoot(double val);

	// regression
	static Eigen::Vector3f linearRegression(Eigen::MatrixXf A, Eigen::VectorXf b);
	static void linearRegression(const std::vector<float> &X, const std::vector<float> &Y, float &slope, float &intercept);
};

#endif // UTILITY_H

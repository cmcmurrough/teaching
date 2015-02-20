/***********************************************************************************************************************
 * @FILE Utility.cpp
 * @BRIEF Implementation of Utility class
 *
 * This class contains commonly used utility functions and helper classes
 *
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/

#include "Utility.h"
#include <stdio.h>
#include <fcntl.h>
#include <io.h>
#include <iostream>
#include <fstream>
#include <boost/chrono.hpp>
#include <boost/math/constants/constants.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigen>

// include platform-specific files
#if defined _WIN64 || defined _WIN32
#define NOMINMAX
#include <windows.h>
#elif __linux
#endif

/***********************************************************************************************************************
 * @BRIEF Creates a console for printing during execution
 *
 * Creates a console window and redirects stdout. Resulting std::cout will print to this console. This console can be
 * used in windows applications that do not automatically include a console, such services
 *
 * @PARAM[in] maxConsoleLines the number of lines to store in the console buffer (default:100)
 * @AUTHOR Christopher D. McMurrough
 * @NOTE code derived from http://www.halcyon.com/~ast/dload/guicon.htm
 **********************************************************************************************************************/
void Utility::redirectIOToConsole(int maxConsoleLines)
{
	// this function is windows specific, disable if necessary
	#if defined _WIN64 || defined _WIN32

    int hConHandle;
    long lStdHandle;
    CONSOLE_SCREEN_BUFFER_INFO coninfo;
    FILE *fp;

    // allocate a console for this app
    AllocConsole();

    // set the screen buffer to be big enough to let us scroll text
    GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE),&coninfo);
    coninfo.dwSize.Y = maxConsoleLines;
    SetConsoleScreenBufferSize(GetStdHandle(STD_OUTPUT_HANDLE),coninfo.dwSize);

    // redirect unbuffered STDOUT to the console
    lStdHandle = (long)GetStdHandle(STD_OUTPUT_HANDLE);
    hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);

    fp = _fdopen( hConHandle, "w" );
    *stdout = *fp;
    setvbuf( stdout, NULL, _IONBF, 0 );

    // redirect unbuffered STDIN to the console
    lStdHandle = (long)GetStdHandle(STD_INPUT_HANDLE);
    hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);

    fp = _fdopen( hConHandle, "r" );
    *stdin = *fp;
    setvbuf( stdin, NULL, _IONBF, 0 );

    // redirect unbuffered STDERR to the console
    lStdHandle = (long)GetStdHandle(STD_ERROR_HANDLE);
    hConHandle = _open_osfhandle(lStdHandle, _O_TEXT);

    fp = _fdopen( hConHandle, "w" );
    *stderr = *fp;
    setvbuf( stderr, NULL, _IONBF, 0 );

    // redirect cout, wcout, cin, wcin, wcerr, cerr, wclog and clog
    std::ios::sync_with_stdio();

	#endif
}

/***********************************************************************************************************************
 * @BRIEF prints a message to console in color
 * @PARAM[in] color color to print the message
 * @PARAM[in] msg printf formatted message string
 * @PARAM[in] ... the additional arguments passed to printf
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
void Utility::printColored(char color, const char* msg, ...)
{
	// get the printf arguments
	char buffer[256];
	va_list args;
	va_start(args, msg);
	vsprintf(buffer, msg, args);
	va_end(args);

	// change print color using windows std output handles
	#if defined _WIN64 || defined _WIN32 

	// store the old settings so we can restore when done
	HANDLE hstdout = GetStdHandle( STD_OUTPUT_HANDLE );
	CONSOLE_SCREEN_BUFFER_INFO csbi;
	GetConsoleScreenBufferInfo(hstdout, &csbi);

	// set the color accordingly
	switch(color)
	{
		case 'b':
		case 'B':
			SetConsoleTextAttribute(hstdout, 0x09);
			break;
		case 'g':
		case 'G':
			SetConsoleTextAttribute(hstdout, 0x0A);
			break;
		case 'c':
		case 'C':
			SetConsoleTextAttribute(hstdout, 0x0B);
			break;
		case 'r':
		case 'R':
			SetConsoleTextAttribute(hstdout, 0x0C);
			break;
		case 'm':
		case 'M':
			SetConsoleTextAttribute(hstdout, 0x0D);
			break;
		case 'y':
		case 'Y':
			SetConsoleTextAttribute(hstdout, 0x0E);
			break;
		case 'w':
		case 'W':
			SetConsoleTextAttribute(hstdout, 0x0F);
			break;
		default:
			break;
	}

	// print the message
	printf(buffer);

    // restore the old settings
	SetConsoleTextAttribute(hstdout, csbi.wAttributes);

	// handle OS other than windows
	#else 
	
	#define ANSI_COLOR_RED "\x1b[31m"
	#define ANSI_COLOR_GREEN "\x1b[32m"
	#define ANSI_COLOR_YELLOW "\x1b[33m"
	#define ANSI_COLOR_BLUE "\x1b[34m"
	#define ANSI_COLOR_MAGENTA "\x1b[35m"
	#define ANSI_COLOR_CYAN "\x1b[36m"
	#define ANSI_COLOR_RESET "\x1b[0m"

	// set the color accordingly
	switch(color)
	{
		case 'b':
		case 'B':
			printf("%s%s%s", ANSI_COLOR_BLUE, buffer, ANSI_COLOR_RESET);
			break;
		case 'g':
		case 'G':
			printf("%s%s%s", ANSI_COLOR_GREEN, buffer, ANSI_COLOR_RESET);
			break;
		case 'c':
		case 'C':
			printf("%s%s%s", ANSI_COLOR_CYAN, buffer, ANSI_COLOR_RESET);
			break;
		case 'r':
		case 'R':
			printf("%s%s%s", ANSI_COLOR_RED, buffer, ANSI_COLOR_RESET);
			break;
		case 'm':
		case 'M':
			printf("%s%s%s", ANSI_COLOR_MAGENTA, buffer, ANSI_COLOR_RESET);
			break;
		case 'y':
		case 'Y':
			printf("%s%s%s", ANSI_COLOR_YELLOW, buffer, ANSI_COLOR_RESET);
			break;
		default:
			printf(buffer);
			break;
	}

	return;

	#endif
}

/***********************************************************************************************************************
 * @BRIEF saves the contents of a string buffer to the specified file
 * @PARAM[in] fileName name of the file to save
 * @PARAM[in] ss string buffer containing the file contents
 * @RETURN true if the file was saved successfully
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
bool Utility::writeFile(const char* fileName, std::stringstream &ss)
{
	// attempt to open the output file
	std::ofstream outFile(fileName);

	// write the file
	if(outFile.is_open())
	{
		outFile << ss.rdbuf();
		outFile.close();
		return true;
	}
	else
	{
		return false;
	}
}

/***********************************************************************************************************************
 * @BRIEF Convert a roll, pitch, yaw rotation angles to quaternion
 *
 * Performs the conversion to a quaternion in xyzw form, returning a Quaternionf structure
 *
 * @PARAM[in] roll the roll rotation angle (phi, about x-axis) in radians
 * @PARAM[in] pitch the pitch rotation angle (theta, about y-axis) in radians
 * @PARAM[in] yaw the yaw rotation angle (psi, about z-axis) in radians
 * @RETURN the resulting quaternion
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
Eigen::Quaternionf Utility::rpy2quat(double roll, double pitch, double yaw)
{
	Eigen::Quaternionf q;
	Eigen::AngleAxisf aaZ(static_cast<float>(yaw), Eigen::Vector3f::UnitZ());
	Eigen::AngleAxisf aaY(static_cast<float>(pitch), Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf aaX(static_cast<float>(roll), Eigen::Vector3f::UnitX());
	q = aaZ * aaY * aaX;
	
	return q;
}

/***********************************************************************************************************************
 * @BRIEF Convert a roll, pitch, yaw rotation to quaternion
 *
 * Performs the conversion to a quaternion in xyzw form.
 *
 * @PARAM[in] roll the roll rotation angle (phi, about x-axis) in radians
 * @PARAM[in] pitch the pitch rotation angle (theta, about y-axis) in radians
 * @PARAM[in] yaw the yaw rotation angle (psi, about z-axis) in radians
 * @PARAM[out] qw the w component of the rotation quaternion
 * @PARAM[out] qx the x component of the rotation quaternion
 * @PARAM[out] qy the y component of the rotation quaternion
 * @PARAM[out] qz the z component of the rotation quaternion
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
void Utility::rpy2quat(double roll, double pitch, double yaw, double &qw, double &qx, double &qy, double &qz)
{
	Eigen::Quaternionf q = Utility::rpy2quat(roll, pitch, yaw);
	qw = static_cast<double>(q.w());
	qx = static_cast<double>(q.x());
	qy = static_cast<double>(q.y());
	qz = static_cast<double>(q.z());
}

/***********************************************************************************************************************
 * @BRIEF Convert a roll, pitch, yaw rotation to quaternion
 *
 * Performs the conversion to a quaternion in xyzw form, returning a Quaternionf structure
 *
 * @PARAM[in] roll the roll rotation angle in radians
 * @PARAM[in] pitch the pitch rotation angle in radians
 * @RETURN the resulting quaternion
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
Eigen::Quaternionf Utility::affine2quat(const Eigen::Affine3f &transformation)
{
	// extract the quaternion rotation
	Eigen::Matrix3f rotation = transformation.rotation();
	Eigen::Quaternionf quat(rotation);
	return quat;
}

/***********************************************************************************************************************
 * @BRIEF Converts a quaternion rotation and translation vector to an affine transformation
 * @PARAM[in] position the translation vector of the transformation
 * @PARAM[in] orientation the quaterntion rotation of the transformation
 * @RETURN the affine transformation matrix
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
Eigen::Affine3f Utility::quat2affine(const Eigen::Vector4f &position, const Eigen::Quaternionf &orientation)
{
	Eigen::Matrix3f rotation = orientation.normalized().toRotationMatrix();
	Eigen::Affine3f t;

	t(0,0) = rotation(0,0);
	t(0,1) = rotation(0,1);
	t(0,2) = rotation(0,2);
	t(0,3) = position[0];
	t(1,0) = rotation(1,0);
	t(1,1) = rotation(1,1);
	t(1,2) = rotation(1,2);
	t(1,3) = position[1];
	t(2,0) = rotation(2,0);
	t(2,1) = rotation(2,1);
	t(2,2) = rotation(2,2);
	t(2,3) = position[2];
	t(3,0) = 0;
	t(3,1) = 0;
	t(3,2) = 0;
	t(3,3) = 1;

	return t;
}

/***********************************************************************************************************************
 * @BRIEF Converts a roll, pitch, yaw rotation to a homogenous rotation matrix
 * @PARAM[in] roll the roll rotation angle in radians (rotation about x)
 * @PARAM[in] pitch the pitch rotation angle in radians (rotation about y)
 * @PARAM[in] yaw the yaw rotation angle in radians (rotation about z)
 * @RETURN the homogenous transformation matrix
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
Eigen::Matrix3f Utility::rpy2mat(double roll, double pitch, double yaw)
{
    Eigen::Matrix3f R_z, R_y, R_x, R_zyx;

    R_z << cos(static_cast<float>(yaw)), -sin(static_cast<float>(yaw)), 0, sin(static_cast<float>(yaw)), cos(static_cast<float>(yaw)), 0, 0, 0, 1;
    R_y << cos(static_cast<float>(pitch)), 0, sin(static_cast<float>(pitch)), 0, 1, 0, -sin(static_cast<float>(pitch)), 0, cos(static_cast<float>(pitch));
    R_x << 1, 0, 0, 0, cos(static_cast<float>(roll)), -sin(static_cast<float>(roll)), 0, sin(static_cast<float>(roll)), cos(static_cast<float>(roll));
    R_zyx = R_z * R_y * R_x;

    return R_zyx;
}

/***********************************************************************************************************************
 * @BRIEF Converts a homogenous rotation matrix to roll, pitch, yaw angles
 * @PARAM[in] rotation the input homogenous rotation matrix
 * @PARAM[out] roll the roll rotation angle in radians (rotation about x)
 * @PARAM[out] pitch the pitch rotation angle in radians (rotation about y)
 * @PARAM[out] yaw the yaw rotation angle in radians (rotation about z)
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
void Utility::mat2rpy(const Eigen::Matrix3f &rotation, double &roll, double &pitch, double &yaw)
{
	Eigen::Vector3f angles = rotation.eulerAngles(2, 1, 0);

	roll = static_cast<double>(angles[2]);
	pitch = static_cast<double>(angles[1]);
	yaw = static_cast<double>(angles[0]);
}

/***********************************************************************************************************************
 * @BRIEF Converts a position vector and roll, pitch, yaw rotation to an affine transformation matrix
 * @PARAM[in] position the translation vector of the transformation
 * @PARAM[in] roll the roll rotation angle in radians (rotation about x)
 * @PARAM[in] pitch the pitch rotation angle in radians (rotation about y)
 * @PARAM[in] yaw the yaw rotation angle in radians (rotation about z)
 * @RETURN the affine transformation matrix
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
Eigen::Affine3f Utility::xyzrpy2affine(const Eigen::Vector4f &position, double roll, double pitch, double yaw)
{
	Eigen::Matrix3f rotation = Utility::rpy2mat(roll, pitch, yaw);
	Eigen::Affine3f t;

	t(0,0) = rotation(0,0);
	t(0,1) = rotation(0,1);
	t(0,2) = rotation(0,2);
	t(0,3) = position[0];
	t(1,0) = rotation(1,0);
	t(1,1) = rotation(1,1);
	t(1,2) = rotation(1,2);
	t(1,3) = position[1];
	t(2,0) = rotation(2,0);
	t(2,1) = rotation(2,1);
	t(2,2) = rotation(2,2);
	t(2,3) = position[2];
	t(3,0) = 0;
	t(3,1) = 0;
	t(3,2) = 0;
	t(3,3) = 1;

	return t;
}

/***********************************************************************************************************************
 * @BRIEF Converts x, y, z coordinates and roll, pitch, yaw rotations to an affine transformation matrix
 * @PARAM[in] x the x coordinate of the translation
 * @PARAM[in] y the y coordinate of the translation
 * @PARAM[in] z the z coordinate of the translation
 * @PARAM[in] roll the roll rotation angle in radians (rotation about x)
 * @PARAM[in] pitch the pitch rotation angle in radians (rotation about y)
 * @PARAM[in] yaw the yaw rotation angle in radians (rotation about z)
 * @RETURN the affine transformation matrix
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
Eigen::Affine3f Utility::xyzrpy2affine(double x, double y, double z, double roll, double pitch, double yaw)
{
	Eigen::Matrix3f rotation = Utility::rpy2mat(roll, pitch, yaw);
	Eigen::Affine3f t;

	t(0,0) = rotation(0,0);
	t(0,1) = rotation(0,1);
	t(0,2) = rotation(0,2);
	t(0,3) = static_cast<float>(x);
	t(1,0) = rotation(1,0);
	t(1,1) = rotation(1,1);
	t(1,2) = rotation(1,2);
	t(1,3) = static_cast<float>(y);
	t(2,0) = rotation(2,0);
	t(2,1) = rotation(2,1);
	t(2,2) = rotation(2,2);
	t(2,3) = static_cast<float>(z);
	t(3,0) = 0;
	t(3,1) = 0;
	t(3,2) = 0;
	t(3,3) = 1;

	return t;
}

/***********************************************************************************************************************
 * @BRIEF Converts x, y, z coordinates x, y, z basis vectors to an affine transformation matrix
 * @PARAM[in] position the translation vector of the transformation
 * @PARAM[in] x the x axis direction vector
 * @PARAM[in] y the y axis direction vector
 * @PARAM[in] z the z axis direction vector
 * @RETURN the affine transformation matrix
 * @NOTE the x, y, and z basis vectors must be orthonormalized
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
Eigen::Affine3f Utility::basis2affine(const Eigen::Vector4f &position, const Eigen::Vector4f &x, const Eigen::Vector4f &y, const Eigen::Vector4f &z)
{
	Eigen::Affine3f t;

	t(0,0) = x(0);
	t(0,1) = y(0);
	t(0,2) = z(0);
	t(0,3) = position(0);
	t(1,0) = x(1);
	t(1,1) = y(1);
	t(1,2) = z(1);
	t(1,3) = position(1);
	t(2,0) = x(2);
	t(2,1) = y(2);
	t(2,2) = z(2);
	t(2,3) = position(2);
	t(3,0) = 0;
	t(3,1) = 0;
	t(3,2) = 0;
	t(3,3) = 1;

	return t;
}


/***********************************************************************************************************************
 * @BRIEF Converts an Affine3f transformation matrix to a homogeneous Matrix4f
 * @PARAM[in] transformation the affine transformation matrix
 * @RETURN the homogenous transformation matrix
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
Eigen::Matrix4f Utility::affine2mat(const Eigen::Affine3f &transformation)
{
	Eigen::Matrix4f mat(transformation.matrix());
	return mat;
}

/***********************************************************************************************************************
 * @BRIEF converts the cartesian coordinates to spherical
 * @PARAM[in] x the x coordinate
 * @PARAM[in] y the y coordinate
 * @PARAM[in] z the z coordinate
 * @PARAM[out] radius the radius in spherical coordinates
 * @PARAM[out] inclination the inclination angle (theta) in spherical coordinates
 * @PARAM[out] azimuth the azimuth angle (phi) in spherical coordinates
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
void Utility::cartesian2Spherical(double x, double y, double z, double &radius, double &inclination, double &azimuth)
{
    radius = sqrt(x*x + y*y + z*z);
	inclination = std::acos(z / radius);
	azimuth = std::atan2(y, x);
}

/***********************************************************************************************************************
 * @BRIEF Convert degress to radians
 * @PARAM[in] degrees the angle expressed in degrees
 * @RETURN the converted angle in radians
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
double Utility::deg2rad(double degrees)
{
	return (degrees * 0.017453293);
}

/***********************************************************************************************************************
 * @BRIEF Convert radians to degrees
 * @PARAM[in] radians the angle expressed in radians
 * @RETURN the converted angle in degrees
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
double Utility::rad2deg(double radians)
{
	return (radians * 57.29578);
}

/***********************************************************************************************************************
 * @BRIEF Convert a vector3f to vector4f
 * @PARAM[in] lastVal the value to truncate to vec (default:0)
 * @RETURN the vector4f representation of vec
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
Eigen::Vector4f Utility::toVec4(Eigen::Vector3f &vec, float lastVal)
{
	return Eigen::Vector4f(vec[0], vec[1], vec[2], lastVal);
}

/***********************************************************************************************************************
 * @BRIEF computes the distance of a point to a line in 3D
 * @PARAM[in] p position of the point
 * @PARAM[in] line_p1 position of the first point
 * @PARAM[in] line_p2 position of the second point
 * @RETURN minimum distance of the point to the line
 * @AUTHOR Christoper D. McMurrough
 * @NOTE line_pt and line_dir define the infinitely long long in 3D space (they do not define a line segment)
 * @WARNING p[3], line_pt[3], and line_dir[3] should be equal to 0
 **********************************************************************************************************************/
double Utility::pointToLineDistance(const Eigen::Vector4f &p, const Eigen::Vector4f &line_p1, const Eigen::Vector4f &line_p2)
{
    // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p1-p0)) / norm(p2-p1)
    double numerator = (p - line_p1).cross3(p - line_p2).norm();
    double denominator = (line_p2 - line_p1).norm();
    return numerator / denominator;
}

/***********************************************************************************************************************
 * @BRIEF computes the distance of a point to a line in 3D
 * @PARAM[in] px x coordinate of the point
 * @PARAM[in] py y coordinate of the point
 * @PARAM[in] pz z coordinate of the point
 * @PARAM[in] line_p1x x coordinate of the first line point
 * @PARAM[in] line_p1y y coordinate of the first line point
 * @PARAM[in] line_p1z z coordinate of the first line point
 * @PARAM[in] line_p2x x coordinate of the second line point
 * @PARAM[in] line_p2y y coordinate of the second line point
 * @PARAM[in] line_p2z z coordinate of the second line point
 * @RETURN minimum distance of the point to the line
 * @AUTHOR Christoper D. McMurrough
 * @NOTE line_pt and line_dir define the infinitely long long in 3D space (they do not define a line segment)
 **********************************************************************************************************************/
double Utility::pointToLineDistance(double px, double py, double pz, double line_p1x, double line_p1y, double line_p1z, double line_p2x, double line_p2y, double line_p2z)
{
	Eigen::Vector4f p(static_cast<float>(px), static_cast<float>(py), static_cast<float>(pz), 0);
	Eigen::Vector4f line_p1(static_cast<float>(line_p1x), static_cast<float>(line_p1y), static_cast<float>(line_p1z), 0);
	Eigen::Vector4f line_p2(static_cast<float>(line_p2x), static_cast<float>(line_p2y), static_cast<float>(line_p2z), 0);

    // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p1-p0)) / norm(p2-p1)
    double numerator = (p - line_p1).cross3(p - line_p2).norm();
    double denominator = (line_p2 - line_p1).norm();
    return numerator / denominator;
}

/***********************************************************************************************************************
 * @BRIEF computes the distance of a point to a line segment in 3D
 * @PARAM[in] p position of the point
 * @PARAM[in] line_p1 position of the first point
 * @PARAM[in] line_p2 position of the second point
 * @RETURN minimum distance of the point to the line segment
 * @AUTHOR Christoper D. McMurrough
 * @WARNING p[3], line_pt[3], and line_dir[3] should be equal to 0
 * @NOTE code derrived from http://geomalgorithms.com/a02-_lines.html
 * @TODO verify that this works in all cases
 **********************************************************************************************************************/
double Utility::pointToLineSegmentDistance(const Eigen::Vector4f &p, const Eigen::Vector4f &line_p1, const Eigen::Vector4f &line_p2)
{
	Eigen::Vector4f v = line_p2 - line_p1;
	Eigen::Vector4f w = p - line_p1;

	double c1 = w.dot(v);
	if(c1 <= 0)
	{
		return Utility::euclideanDistance(p, line_p1);
	}
	double c2 = v.dot(v);
	if(c2 <= c1)
	{
		return Utility::euclideanDistance(p, line_p2);
	}
	float b = static_cast<float>(c1 / c2);
	Eigen::Vector4f pb = line_p1 + b * v;
	return Utility::euclideanDistance(p, pb);
}

/***********************************************************************************************************************
 * @BRIEF computes the distance of a point to a line segment in 3D
 * @PARAM[in] px x coordinate of the point
 * @PARAM[in] py y coordinate of the point
 * @PARAM[in] pz z coordinate of the point
 * @PARAM[in] line_p1x x coordinate of the first line point
 * @PARAM[in] line_p1y y coordinate of the first line point
 * @PARAM[in] line_p1z z coordinate of the first line point
 * @PARAM[in] line_p2x x coordinate of the second line point
 * @PARAM[in] line_p2y y coordinate of the second line point
 * @PARAM[in] line_p2z z coordinate of the second line point
 * @RETURN minimum distance of the point to the line segment
 * @AUTHOR Christoper D. McMurrough
 * @NOTE code derrived from http://geomalgorithms.com/a02-_lines.html
 * @TODO verify that this works in all cases
 **********************************************************************************************************************/
double Utility::pointToLineSegmentDistance(double px, double py, double pz, double line_p1x, double line_p1y, double line_p1z, double line_p2x, double line_p2y, double line_p2z)
{
	Eigen::Vector4f p(static_cast<float>(px), static_cast<float>(py), static_cast<float>(pz), 0);
	Eigen::Vector4f line_p1(static_cast<float>(line_p1x), static_cast<float>(line_p1y), static_cast<float>(line_p1z), 0);
	Eigen::Vector4f line_p2(static_cast<float>(line_p2x), static_cast<float>(line_p2y), static_cast<float>(line_p2z), 0);

    return Utility::pointToLineSegmentDistance(p, line_p1, line_p2);
}

/***********************************************************************************************************************
 * @BRIEF computes the distance of a point to a ray in 3D
 * @PARAM[in] p position of the point
 * @PARAM[in] origin position of the ray origin
 * @PARAM[in] dir direction of the ray
 * @RETURN minimum distance of the point to the ray
 * @AUTHOR Christoper D. McMurrough
 * @WARNING p[3], origin[3], and dir[3] should be equal to 0
 * @NOTE code derrived from http://geomalgorithms.com/a02-_lines.html
 * @TODO verify that this works in all cases
 **********************************************************************************************************************/
double Utility::pointToRayDistance(const Eigen::Vector4f &p, const Eigen::Vector4f &origin, const Eigen::Vector4f &dir)
{
	Eigen::Vector4f v = dir;
	Eigen::Vector4f w = p - origin;

	double c1 = w.dot(v);
	if(c1 <= 0)
	{
		return Utility::euclideanDistance(p, origin);
	}
	double c2 = v.dot(v);
	float b = static_cast<float>(c1 / c2);
	Eigen::Vector4f pb = origin + b * v;
	return Utility::euclideanDistance(p, pb);
}

/***********************************************************************************************************************
 * @BRIEF computes the distance of a point to a line segment in 3D
 * @PARAM[in] px x coordinate of the point
 * @PARAM[in] py y coordinate of the point
 * @PARAM[in] pz z coordinate of the point
 * @PARAM[in] origin_x x coordinate of the ray origin
 * @PARAM[in] origin_y y coordinate of the ray origin
 * @PARAM[in] origin_z z coordinate of the ray origin
 * @PARAM[in] dir_x x coordinate of the ray direction
 * @PARAM[in] dir_y y coordinate of the ray direction
 * @PARAM[in] dir_z z coordinate of the ray direction
 * @RETURN minimum distance of the point to the line segment
 * @AUTHOR Christoper D. McMurrough
 * @NOTE code derrived from http://geomalgorithms.com/a02-_lines.html
 * @TODO verify that this works in all cases
 **********************************************************************************************************************/
double Utility::pointToRayDistance(double px, double py, double pz, double origin_x, double origin_y, double origin_z, double dir_x, double dir_y, double dir_z)
{
	Eigen::Vector4f p(static_cast<float>(px), static_cast<float>(py), static_cast<float>(pz), 0);
	Eigen::Vector4f origin(static_cast<float>(origin_x), static_cast<float>(origin_y), static_cast<float>(origin_z), 0);
	Eigen::Vector4f dir(static_cast<float>(dir_x), static_cast<float>(dir_y), static_cast<float>(dir_z), 0);

    return Utility::pointToLineSegmentDistance(p, origin, dir);
}

/***********************************************************************************************************************
 * @BRIEF computes the distance of a point to a plane in 3D
 * @PARAM[in] p position of the point
 * @PARAM[in] plane the coefficients of the plane in the form ax+by+cz+d=0
 * @RETURN minimum distance of the point to the line
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
double Utility::pointToPlaneDistance(const Eigen::Vector4f &p, const Eigen::Vector4f &plane)
{
	Eigen::Vector3f x(p[0], p[1], p[2]);
	Eigen::Vector3f v(plane[0], plane[1], plane[2]);

	// compute the point to plane distance
	return v.normalized().dot(x) + (plane[3] / v.norm());
}

/***********************************************************************************************************************
 * @BRIEF computes the distance of a point to a line in 3D
 * @PARAM[in] px x coordinate of the point
 * @PARAM[in] py y coordinate of the point
 * @PARAM[in] pz z coordinate of the point
 * @PARAM[in] a the a coefficient of the plane in the form ax+by+cz+d=0
 * @PARAM[in] b the b coefficient of the plane in the form ax+by+cz+d=0
 * @PARAM[in] c the c coefficient of the plane in the form ax+by+cz+d=0
 * @PARAM[in] d the d coefficient of the plane in the form ax+by+cz+d=0
 * @RETURN minimum distance of the point to the plane
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
double Utility::pointToPlaneDistance(double px, double py, double pz, double a, double b, double c, double d)
{
	double numerator = fabs(a*px + b*py + c*pz + d);
	double denominator = sqrt(a*a + b*b + c*c);
    return numerator / denominator;
}

/***********************************************************************************************************************
 * @BRIEF computes the distance along a ray to a plane
 * @PARAM[in] origin the ray origin point
 * @PARAM[in] direction the direction vector of the ray
 * @PARAM[in] plane the coefficients of the plane in the form ax+by+cz+d=0
 * @RETURN the distance along the ray to the plane
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
double Utility::rayToPlaneDistance(const Eigen::Vector4f &origin, const Eigen::Vector4f &direction, const Eigen::Vector4f &plane)
{
	Eigen::Vector4f rayDir(direction[0], direction[1], direction[2], 0);
	Eigen::Vector4f planeNormal(plane[0], plane[1], plane[2], 0);
	Eigen::Vector4f planePoint = Eigen::Vector4f(0, 0, 0, 0) + static_cast<float>(plane[3])*((-planeNormal).normalized());

	return planeNormal.dot(planeNormal - direction) / (planeNormal.dot(rayDir));
}

/***********************************************************************************************************************
 * @BRIEF computes the euclidean distance between two points
 * @PARAM[in] p1 position of the first point
 * @PARAM[in] p2 position of the second point
 * @RETURN euclidean distance separating the two points
 * @AUTHOR Christoper D. McMurrough
 * @WARNING p1[3] and p2[3] should be equal to 0
 **********************************************************************************************************************/
double Utility::euclideanDistance(const Eigen::Vector4f &p1, const Eigen::Vector4f &p2)
{
    // D = ||(P2-P1)||
	//return ((p2 - p1).norm());
	return ((Eigen::Vector4f(p2[0], p2[1], p2[2], 0) - Eigen::Vector4f(p1[0], p1[1], p1[2], 0)).norm());
}

/***********************************************************************************************************************
 * @BRIEF computes the euclidean distance between two points
 * @PARAM[in] p1x x coordinate of the first point
 * @PARAM[in] p1y y coordinate of the first point
 * @PARAM[in] p1z z coordinate of the first point
 * @PARAM[in] p2x x coordinate of the second point
 * @PARAM[in] p2y y coordinate of the second point
 * @PARAM[in] p2z z coordinate of the second point
 * @RETURN euclidean distance separating the two points
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
double Utility::euclideanDistance(double p1x, double p1y, double p1z, double p2x, double p2y, double p2z)
{
    // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p1-p0)) / norm(p2-p1)
	return sqrt((p1x - p2x)*(p1x - p2x) + (p1y - p2y)*(p1y - p2y) + (p1z - p2z)*(p1z - p2z));
}

/***********************************************************************************************************************
 * @BRIEF computes a point lying on the line defined by line_pt and line_dir a given distance from line_pt
 * @PARAM[in] line_p1 the first point on the line
 * @PARAM[in] line_p2 the second point on the line
 * @PARAM[in] distance between line_p1 and p
 * @PARAM[out] p resulting point on the line a distance of d from line_p1
 * @AUTHOR Christoper D. McMurrough
 * @WARNING line_pt[3] and line_dir[3] should be equal to 0
 **********************************************************************************************************************/
void Utility::pointAlongLine(const Eigen::Vector4f &line_p1, const Eigen::Vector4f &line_p2, double d, Eigen::Vector4f &p)
{
    // p = P0 + d*(||P1-P0||)
    p = line_p1 + static_cast<float>(d)*((line_p2 - line_p1).normalized());
}

/***********************************************************************************************************************
 * @BRIEF computes a point lying on the ray defined by an origin point and direction vector
 * @PARAM[in] origin the origin of the ray
 * @PARAM[in] direction the direction vector of the ray
 * @PARAM[in] distance between line_p1 and p
 * @PARAM[out] p resulting point lying on the ray a distance of d from the origin
 * @AUTHOR Christoper D. McMurrough
 * @WARNING origin[3] and direction[3] should be equal to 0
 **********************************************************************************************************************/
void Utility::pointAlongRay(const Eigen::Vector4f &origin, const Eigen::Vector4f &direction, double d, Eigen::Vector4f &p)
{
    p = origin + static_cast<float>(d)*((direction).normalized());
}

/***********************************************************************************************************************
 * @BRIEF computes a point lying on the line defined by lp and ld with a given distance from l1
 * @PARAM[in] line_p1x x coordinate of the first line point
 * @PARAM[in] line_p1y y coordinate of the first line point
 * @PARAM[in] line_p1z z coordinate of the first line point
 * @PARAM[in] line_p2x x coordinate of the second line point
 * @PARAM[in] line_p2y y coordinate of the second line point
 * @PARAM[in] line_p2z z coordinate of the second line point
 * @PARAM[in] distance between lp and p
 * @PARAM[out] px resulting x coordinate of point on the line a distance of d from lp
 * @PARAM[out] py resulting y coordinate of point on the line a distance of d from lp
 * @PARAM[out] pz resulting z coordinate of point on the line a distance of d from lp
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
void Utility::pointAlongLine(double line_p1x, double line_p1y, double line_p1z, double line_p2x, double line_p2y, double line_p2z, double d, double &px, double &py, double &pz)
{
	Eigen::Vector4f p;
	Eigen::Vector4f line_p1(static_cast<float>(line_p1x), static_cast<float>(line_p1y), static_cast<float>(line_p1z), 0);
	Eigen::Vector4f line_p2(static_cast<float>(line_p2x), static_cast<float>(line_p2y), static_cast<float>(line_p2z), 0);

	pointAlongLine(line_p1, line_p2, d, p);
	px = p[0];
	py = p[1];
	pz = p[2];
}

/***********************************************************************************************************************
 * @BRIEF distance of point to a tetragon edge in 3D
 *
 * computes the minimum distance of a given point to one of the edges formed by the vertices, assigned clockwise
 *
 * @PARAM[in] p query point
 * @PARAM[in] v1 the first vertex
 * @PARAM[in] v2 the first vertex
 * @PARAM[in] v3 the first vertex
 * @PARAM[in] v4 the first vertex
 * @RETURN[in] minimum distance of the point to one of the edges
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
double Utility::distanceToTetragon(const Eigen::Vector4f &p, const Eigen::Vector4f &v1, const Eigen::Vector4f &v2, const Eigen::Vector4f &v3, const Eigen::Vector4f &v4)
{
    double d[] = {0, 0, 0, 0};

    d[0] = Utility::pointToLineDistance(p, v1, v2);
    d[1] = Utility::pointToLineDistance(p, v2, v3);
    d[2] = Utility::pointToLineDistance(p, v3, v4);
    d[3] = Utility::pointToLineDistance(p, v4, v1);

    return *std::min_element(d, d + 4);
}

/***********************************************************************************************************************
 * @BRIEF compute the intersection point between two lines
 *
 * Computes the intersection point between two lines. If the lines are skew, the closest point between the lines is
 * given. In either case, the minimum distance between the lines is returned.
 *
 * @PARAM[in] line1_p1 the first point on the first line
 * @PARAM[in] line1_p2 the second point on the first line
 * @PARAM[in] line2_p1 the first point on the second line
 * @PARAM[in] line2_p2 the second point on the second line
 * @PARAM[out] intersection the closest point between the two lines
 * @RETURN minimum distance between the two lines
 * @TODO verify that this works as expected for all cases, and change p2s to direction vectors instead of points
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
double Utility::lineIntersection(const Eigen::Vector4f &line1_p1, const Eigen::Vector4f &line1_p2, const Eigen::Vector4f &line2_p1, const Eigen::Vector4f &line2_p2, Eigen::Vector4f &intersection)
{
	const double SMALL_NUM = 0.00000001;

	Eigen::Vector4f u = line1_p2 - line1_p1;
	Eigen::Vector4f v = line2_p2 - line2_p1;
	Eigen::Vector4f w = line1_p1 - line2_p1;

	double a = Utility::dotProduct(u, u);
	double b = Utility::dotProduct(u, v);
	double c = Utility::dotProduct(v, v);
	double d = Utility::dotProduct(u, w);
	double e = Utility::dotProduct(v, w);

	double D = a*c - b*b;
	double sc, tc;

    // compute the line parameters of the two closest points
    if (D < SMALL_NUM)
	{
		// the lines are almost parallel, use the largest denominator
        sc = 0.0;
        tc = (b>c ? d/b : e/c);
    }
    else
	{
        sc = (b*e - c*d) / D;
        tc = (a*e - b*d) / D;
    }

	// get the closest point on each line to the other line
	Eigen::Vector4f line1_intersection = line1_p1 + (static_cast<float>(sc) * u);
	Eigen::Vector4f line2_intersection = line2_p1 + (static_cast<float>(tc) * v);

	// compute the midpoint
	intersection = (line1_intersection + line2_intersection) * 0.5;
	intersection[3] = 0;

    // return the distance between the two points
	return Utility::euclideanDistance(line1_intersection, line2_intersection);
}

/***********************************************************************************************************************
 * @BRIEF compute the intersection point of 3 planes
 *
 * Computes the intersection point of 3 non-paralell planes, returning false if a single intersection could not be found
 *
 * @PARAM[in] plane1 the set of coefficients for the first plane in the form Ax+By+Cz+D=0
 * @PARAM[in] plane2 the set of coefficients for the first plane in the form Ax+By+Cz+D=0
 * @PARAM[in] plane3 the set of coefficients for the first plane in the form Ax+By+Cz+D=0
 * @PARAM[out] intersection_point the intersection point between the 3 planes
 * @PARAM[in] determinantTolerance the minimum determinant value of which to consider the planes non-paralell (default:0.001);
 * @RETURN true if the 3 planes intersect at a single point
 * @NOTE this function is a dupicate of pcl::threePlanesIntersection, which is currently in pcl-trunk
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
bool Utility::threePlanesIntersection(const Eigen::Vector4f &plane1, const Eigen::Vector4f &plane2, const Eigen::Vector4f &plane3, Eigen::Vector4f &intersection, double determinantTolerance)
{
	// check for parallel planes
	Eigen::Matrix3f normals_in_lines;
	for (int i = 0; i < 3; i++)
	{
		normals_in_lines (i, 0) = plane1[i];
		normals_in_lines (i, 1) = plane2[i];
		normals_in_lines (i, 2) = plane3[i];
	}

	// return false if the determinant is close to zero
	double determinant = normals_in_lines.determinant();
	if (fabs (determinant) < determinantTolerance)
	{
		return false;
	}

	// setup up the left side of the 3 equations
	Eigen::Matrix3f left_member;
	for (int i = 0; i < 3; i++)
	{
		left_member (0, i) = plane1[i];
		left_member (1, i) = plane2[i];
		left_member (2, i) = plane3[i];
	}

	// setup up the right side of the 3 equations
	Eigen::Vector3f right_member;
	right_member << -plane1[3], -plane2[3], -plane3[3];

	// solve the system of equations
	intersection = left_member.fullPivLu ().solve (right_member);
	return (true);
}

/***********************************************************************************************************************
 * @BRIEF compute the intersection point of a line with a plane
 *
 * Computes the intersection point of a line, defined by a point and direction vector, with a plane.
 *
 * @PARAM[in] linePoint a point on the line
 * @PARAM[in] lineDir the direction vector of the line
 * @PARAM[in] plane the set of coefficients for the plane in the form Ax+By+Cz+D=0
 * @PARAM[out] intersection_point the intersection point between line and plane
 * @PARAM[in] paralellTolerance the minimum value of which to consider the plane and line non-paralell (default:0.001);
 * @RETURN true if the line and plane intersect at a single point
 * @NOTE code derrived from http://geomalgorithms.com/a05-_intersect-1.html
 * @TODO verify that this works in all cases
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
bool Utility::linePlaneIntersection(const Eigen::Vector4f &linePoint, const Eigen::Vector4f &lineDir, const Eigen::Vector4f &plane, Eigen::Vector4f &intersection, double paralellTolerance)
{
	Eigen::Vector4f u = lineDir;
	Eigen::Vector4f pn = Eigen::Vector4f(plane[0], plane[1], plane[2], 0);
	Eigen::Vector4f p0 = -pn * plane[3];
	Eigen::Vector4f w = linePoint - p0;

	float D = pn.dot(u);
	float N = -pn.dot(w);

	// check to see if the line and plane are paralell or coplanar
	if(std::fabs(D) < paralellTolerance)
	{
		return false;
	}

	// the line and plane are not paralell, compute the intersection
	float sI = N / D;

	// make sure there is an intersection
	if(sI < 0 || sI > 1)
	{
		return false;
	}

	intersection = linePoint + sI * u;
	return true;
}

/***********************************************************************************************************************
 * @BRIEF gets the coefficients of a plane formed by 3 points
 * @PARAM[in] p1 the first point on the plane
 * @PARAM[in] p2 the second point on the plane
 * @PARAM[in] p3 the third point on the plane
 * @PARAM[out] plane the resulting planar coefficients in the form Ax+By+Cz+D=0
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
void Utility::pointsToPlane(const Eigen::Vector4f &p1, const Eigen::Vector4f &p2, const Eigen::Vector4f &p3, Eigen::Vector4f &plane)
{
	//Eigen::Vector4f planeNormal = ((p2 - p1).cross(p3 - p1)).normalized();
	//plane[0] = planeNormal[0];
	//plane[1] = planeNormal[1];
	//plane[2] = planeNormal[2];
	//plane[3] = -planeNormal.dot(p1);

	Eigen::Vector3f n = Eigen::Vector3f(p2[0] - p1[0], p2[1]- p1[1], p2[2]- p1[2]).cross(Eigen::Vector3f(p3[0] - p1[0], p3[1]- p1[1], p3[2]- p1[2]));
	n.normalize();
	plane[0] = n[0];
	plane[1] = n[1];
	plane[2] = n[2];
	plane[3] = -n.dot(Eigen::Vector3f(p1[0], p1[1], p1[2]));
}

/***********************************************************************************************************************
 * @BRIEF gets the coefficients of a plane formed by a normal and a point
 * @PARAM[in] normal the normal vector of the plane
 * @PARAM[in] p a point lying on the plane
 * @PARAM[out] plane the resulting planar coefficients in the form Ax+By+Cz+D=0
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
void Utility::pointNormalToPlane(const Eigen::Vector4f &normal, const Eigen::Vector4f &p, Eigen::Vector4f &plane)
{
	plane = normal;
	plane[3] = 0;
	plane.normalize();
	plane[3] = - (plane[0] * p[0]) - (plane[1] * p[1]) - (plane[2] * p[2]);
}

/***********************************************************************************************************************
 * @BRIEF returns an orthonormal basis constructed using the gram-schmidt process on the input vectors
 * @PARAM[in] u1 the first input vector
 * @PARAM[in] u2 the second input vector
 * @PARAM[in] u3 the third input vector
 * @PARAM[in] v1 the first basis vector
 * @PARAM[in] v2 the second basis vector
 * @PARAM[in] v3 the third basis vector
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
void Utility::orthonormalBasis(const Eigen::Vector4f &u1, const Eigen::Vector4f &u2, const Eigen::Vector4f &u3, Eigen::Vector4f &v1, Eigen::Vector4f &v2, Eigen::Vector4f &v3)
{
	v1 = u1;
	v2 = u2 - (u2.dot(v1) / v1.squaredNorm()) * v1;
	v3 = u3 - (u3.dot(v1) / v1.squaredNorm()) * v1 - (u3.dot(v2) / v2.squaredNorm()) * v2;

	v1.normalize();
	v2.normalize();
	v3.normalize();
}

/***********************************************************************************************************************
 * @BRIEF Checks to see if a point is above a plane
 *
 * Returns true if the target point lies on the side of the plane on which the normal points
 *
 * @PARAM[in] p the input point
 * @PARAM[in] plane the plane coefficients of the cut plane in the for Ax+By+Cz+D=0
 * @RETURN true if the point lies on the side of the plane in which the normal points
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
bool Utility::isPointAbovePlane(const Eigen::Vector4f &p, const Eigen::Vector4f &plane)
{
	float temp = 0;

	temp = plane[0] * p[0] + plane[1] * p[1] + plane[2] * p[2] + plane[3];
	if(temp < 0)
	{
		return false;
	}
	else
	{
		return false;
	}
}

/***********************************************************************************************************************
 * @BRIEF apply a transformation to a point
 *
 * Applies a homogenous transformation to a point, returning the transformed point
 *
 * @PARAM[in] p the point to be transformed
 * @PARAM[in] transform the homogenous transformation to apply to the point
 * @RETURN the transformed point
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
Eigen::Vector4f Utility::transformPoint(const Eigen::Vector4f &p, const Eigen::Matrix4f &transform)
{
	float x = static_cast<float> (transform (0, 0) * p(0) + transform (0, 1) * p(1) + transform (0, 2) * p(2) + transform (0, 3));
	float y = static_cast<float> (transform (1, 0) * p(0) + transform (1, 1) * p(1) + transform (1, 2) * p(2) + transform (1, 3));
	float z = static_cast<float> (transform (2, 0) * p(0) + transform (2, 1) * p(1) + transform (2, 2) * p(2) + transform (2, 3));

	return Eigen::Vector4f(x, y, z, 0);
}

/***********************************************************************************************************************
 * @BRIEF apply a rotation to a normal vector
 *
 * Applies a rotation matrix to a normal vector, returning the transformed normal
 *
 * @PARAM[in] v the normal to be transformed
 * @PARAM[in] rotation the rotation matrix to apply to the point
 * @RETURN the transformed normal
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
Eigen::Vector3f Utility::rotateNormal(const Eigen::Vector3f &v, const Eigen::Matrix3f &rotation)
{
	Eigen::Vector3f result = (rotation * v.normalized()).normalized();
	return result;
}

/***********************************************************************************************************************
 * @BRIEF computes the gaussian value given distribution parameters
 * @PARAM[in] x the value we wish to evaluate using the gaussian
 * @PARAM[in] a height of the curve peak
 * @PARAM[in] b position of the peak center
 * @PARAM[in] c standard deviation (controls the bell width)
 * @PARAM[in] d vertical offset of the curve
 * @RETURN value of the gaussian function f(x)
 * @AUTHOR Christoper D. McMurrough
 * @NOTE function is an implementation of http://en.wikipedia.org/wiki/Gaussian_function
 **********************************************************************************************************************/
double Utility::gaussian(double x, double a, double b, double c, double d)
{
    // define Euler's constant
    const double e = 2.71828182845904523536028747135266249775;

    // compute and return the gaussian
    return a * pow(e, -(pow(x - b, 2) / pow(2*c, 2))) + d;
}

/***********************************************************************************************************************
 * @BRIEF determines the equality of two double values
 *
 * the machine epsilon has to be scaled to the magnitude of the larger value and multiplied by the desired precision
 * in ULPs (units in the last place)
 *
 * @PARAM[in] a first double value to compare
 * @PARAM[in] b second double value to compare
 * @PARAM[in] ulp precision multiplier (units in the last place)
 * @RETURN true if doubles are equal
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
bool Utility::doubleEquality(double a, double b, int ulp)
{
    return std::fabs(a - b) <= std::numeric_limits<double>::epsilon() * std::max(std::fabs(a), std::fabs(b)) * ulp;
}

/***********************************************************************************************************************
 * @BRIEF check to see if the two doubles are almost equal (if their difference is less than maxDifference
 * @PARAM[in] a first double value to compare
 * @PARAM[in] b second double value to compare
 * @PARAM[in] maxDifference the maximum allowable difference between the two values
 * @RETURN true if doubles are almost equal
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
bool Utility::doubleAlmostEqual(double a, double b, double maxDifference)
{
    return std::fabs(a - b) < maxDifference;
}

/***********************************************************************************************************************
 * @BRIEF check to see if the query point lies within the specified bounding box
 * @PARAM[in] x the query point x coordinate
 * @PARAM[in] y the query point y coordinate
 * @PARAM[in] z the query point z coordinate
 * @PARAM[in] minX the minimum x coordinate
 * @PARAM[in] minY the minimum y coordinate
 * @PARAM[in] minZ the minimum z coordinate
 * @PARAM[in] maxX the maximum x coordinate
 * @PARAM[in] maxY the maximum y coordinate
 * @PARAM[in] maxZ the maximum z coordinate
 * @RETURN true if the point lies within the given bounds
 * @AUTHOR Christopher D. McMurrough
 **********************************************************************************************************************/
bool Utility::isPointWithinRange(double x, double y, double z, double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
{
	// check the x coordinates
	if(x < minX || x > maxX)
	{
		return false;
	}

	// check the y coordinates
	if(y < minY || y > maxY)
	{
		return false;
	}

	// check the z coordinates
	if(z < minZ || z > maxZ)
	{
		return false;
	}

	return true;
}

/***********************************************************************************************************************
 * @BRIEF computes the dot product (inner product) of the two vectors
 * @PARAM[in] p1 the first vector
 * @PARAM[in] p2 the second vector
 * @RETURN the dot product of the two vectors
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
double Utility::dotProduct(const Eigen::Vector4f &p1, const Eigen::Vector4f &p2)
{
    return p1.dot(p2);
}

/***********************************************************************************************************************
 * @BRIEF computes the dot product (inner product) of the two vectors
 * @PARAM[in] p1x x component of the first vector
 * @PARAM[in] p1y y component of the first vector
 * @PARAM[in] p1z z component of the first vector
 * @PARAM[in] p2x x component of the second vector
 * @PARAM[in] p2y y component of the second vector
 * @PARAM[in] p2z z component of the second vector
 * @RETURN the dot product of the two vectors
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
double Utility::dotProduct(double p1x, double p1y, double p1z, double p2x, double p2y, double p2z)
{
	Eigen::Vector4f p1(static_cast<float>(p1x), static_cast<float>(p1y), static_cast<float>(p1z), 0);
	Eigen::Vector4f p2(static_cast<float>(p2x), static_cast<float>(p2y), static_cast<float>(p2z), 0);

	return dotProduct(p1, p2);
}

/***********************************************************************************************************************
 * @BRIEF computes the cross product of the two vectors
 * @PARAM[in] p1 the first vector
 * @PARAM[in] p2 the second vector
 * @RETURN the cross product of the two vectors
 * @NOTE the product is computed as if the input points were of type Eigen::Vector3f
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
Eigen::Vector3f Utility::crossProduct(const Eigen::Vector4f &p1, const Eigen::Vector4f &p2)
{
	Eigen::Vector3f temp1(p1[0], p1[1], p1[2]);
	Eigen::Vector3f temp2(p2[0], p2[1], p2[2]);
    return temp1.cross(temp2);
}

/***********************************************************************************************************************
 * @BRIEF computes the cross product of the two vectors
 * @PARAM[in] p1x x component of the first vector
 * @PARAM[in] p1y y component of the first vector
 * @PARAM[in] p1z z component of the first vector
 * @PARAM[in] p2x x component of the second vector
 * @PARAM[in] p2y y component of the second vector
 * @PARAM[in] p2z z component of the second vector
 * @RETURN the cross product of the two vectors
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
Eigen::Vector3f Utility::crossProduct(double p1x, double p1y, double p1z, double p2x, double p2y, double p2z)
{
	Eigen::Vector4f p1(static_cast<float>(p1x), static_cast<float>(p1y), static_cast<float>(p1z), 0);
	Eigen::Vector4f p2(static_cast<float>(p2x), static_cast<float>(p2y), static_cast<float>(p2z), 0);

	return Utility::crossProduct(p1, p2);
}

/***********************************************************************************************************************
 * @BRIEF computes the square root of a number
 * @PARAM[in] val the number used to compute the square root
 * @RETURN the square root of val
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
double Utility::squareRoot(double val)
{
	return sqrt(val);
}

/***********************************************************************************************************************
 * @BRIEF performs linear regression of the equation Ax=b, returning the least-squares solution to x
 * @PARAM[in] A the matrix of sample values
 * @PARAM[in] b the matrix of result values
 * @RETURN the least-squares solution of x
 * @AUTHOR Christoper D. McMurrough
 * @NOTE code derived from http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
 **********************************************************************************************************************/
Eigen::Vector3f Utility::linearRegression(Eigen::MatrixXf A, Eigen::VectorXf b)
{
	return A.colPivHouseholderQr().solve(b);
}

/***********************************************************************************************************************
 * @BRIEF performs linear regression of the equation y=mx+b, returning the least-squares slope and intercept
 * @PARAM[in] X the vector of x values
 * @PARAM[in] Y the vector of y values
 * @PARAM[out] slope the least-squares slope
 * @PARAM[out] intercept the least-squares intercept
 * @AUTHOR Christoper D. McMurrough
 **********************************************************************************************************************/
void Utility::linearRegression(const std::vector<float> &X, const std::vector<float> &Y, float &slope, float &intercept)
{
	// formulate the problem as the linear system Ax=b
	Eigen::MatrixXf A;
	Eigen::VectorXf b;
	Eigen::VectorXf result;
	int numSamples;

	// check the operands for proper size
	if(Y.size() > 0 && X.size() > 0 && Y.size() == X.size())
	{
		// assemble the eigen structures
		numSamples = static_cast<int>(Y.size());
		A.resize(numSamples, 2);
		b.resize(numSamples);
		for(int i = 0; i < numSamples; i++)
		{
			A(i, 0) = 1;
			A(i, 1) = X.at(i);
			b(i) = Y.at(i);
		}

		// solve the linear system
		result = Utility::linearRegression(A, b);
		intercept = result(0);
		slope = result(1);
	}
	else
	{
		intercept = 0;
		slope = 0;
	}
}


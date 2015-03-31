#ifndef POSE3D
#define POSE3D
// Structure to hold a point
struct Point3d {
	double x;
	double y;
	double z;

	// Constructor sets elements to zero.
	Point3d() { x = 0; y = 0; z = 0; }; 
	Point3d operator+(const Point3d &rhs) const {
		Point3d res;
		res.x = x + rhs.x;res.y = y + rhs.y;res.z = z + rhs.z;
		return res;
	};
	Point3d operator-(const Point3d &rhs) const {
		Point3d res;
		res.x = x - rhs.x;res.y = y - rhs.y;res.z = z - rhs.z;
		return res;
	};
	Point3d operator*(const double &rhs) const {
		Point3d res;
		res.x = x*rhs;res.y = y*rhs;res.z = z*rhs;
		return res;
	};
	Point3d operator/(const double &rhs) const {
		Point3d res;
		res.x = x/rhs;res.y = y/rhs;res.z = z/rhs;
		return res;
	};
	Point3d& operator+=(const Point3d &rhs) {
		x += rhs.x;y += rhs.y;z += rhs.z;
		return *this;
	};
	Point3d& operator-=(const Point3d &rhs) {
		x -= rhs.x;y -= rhs.y;z -= rhs.z;
		return *this;
	};
	Point3d& operator*=(const double &rhs) {
		x = x*rhs;y = y*rhs;z = z*rhs;
		return *this;
	};
	Point3d& operator/=(const double &rhs) {
		x = x/rhs;y = y/rhs;z = z/rhs;
		return *this;
	};
};

// Four quaternion elements define the orientation
struct Orientation3d {
	double a;
	double b;
	double c;
	double d;

	// Constructor sets elements to zero.
	Orientation3d() { a = 0; b = 0; c = 0; d = 0; }; 
	Orientation3d operator+(const Orientation3d &rhs) const {
		Orientation3d res;
		res.a = a + rhs.a;res.b = b + rhs.b;res.c = c + rhs.c;res.d = d + rhs.d;
		return res;
	};
	Orientation3d operator-(const Orientation3d &rhs) const {
		Orientation3d res;
		res.a = a - rhs.a;res.b = b - rhs.b;res.c = c - rhs.c;res.d = d - rhs.d;
		return res;
	};
	Orientation3d operator*(const double &rhs) const {
		Orientation3d res;
		res.a = a*rhs;res.b = b*rhs;res.c = c*rhs;res.d = d*rhs;
		return res;
	};
	Orientation3d operator/(const double &rhs) const {
		Orientation3d res;
		res.a = a/rhs;res.b = b/rhs;res.c = c/rhs;res.d = d/rhs;
		return res;
	};
	Orientation3d& operator+=(const Orientation3d &rhs) {
		a = a + rhs.a;b = b + rhs.b;c = c + rhs.c;d = d + rhs.d;
		return *this;
	};
	Orientation3d& operator-=(const Orientation3d &rhs) {
		a = a - rhs.a;b = b - rhs.b;c = c - rhs.c;d = d - rhs.d;
		return *this;
	};
	Orientation3d& operator*=(const double &rhs) {
		a = a * rhs;b = b * rhs;c = c * rhs;d = d * rhs;
		return *this;
	};
	Orientation3d& operator/=(const double &rhs) {
		a = a / rhs;b = b / rhs;c = c / rhs;d = d / rhs;
		return *this;
	};
	Orientation3d normalize() {
		Orientation3d res;
		double norm = sqrt(a*a+b*b+c*c+d*d);
		res.a = a/norm;res.b = b/norm;res.c = c/norm;res.d = d/norm;
		return res;
	}
};

struct Pose3d {
	Point3d location;
	Orientation3d orientation;
	Pose3d operator+(const Pose3d &rhs) const {
		Pose3d res;
		res.location = location + rhs.location;
		res.orientation = orientation + rhs.orientation;
		return res;
	};
	Pose3d operator-(const Pose3d &rhs) const {
		Pose3d res;
		res.location = location - rhs.location;
		res.orientation = orientation - rhs.orientation;
		return res;
	};
	Pose3d operator*(const double &rhs) const {
		Pose3d res;
		res.location = location * rhs;
		res.orientation = orientation * rhs;
		return res;
	};
	Pose3d operator/(const double &rhs) const {
		Pose3d res;
		res.location = location / rhs;
		res.orientation = orientation / rhs;
		return res;
	};
	Pose3d& operator+=(const Pose3d &rhs) {
		location += rhs.location;
		orientation += rhs.orientation;
		return *this;
	}
	Pose3d& operator-=(const Pose3d &rhs) {
		location -= rhs.location;
		orientation -= rhs.orientation;
		return *this;
	}
	Pose3d& operator/=(const double &rhs) {
		location /= rhs;
		orientation /= rhs;
		return *this;
	}
	Pose3d& operator*=(const double &rhs) {
		location *= rhs;
		orientation *= rhs;
		return *this;
	}
};
#endif

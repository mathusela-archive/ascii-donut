#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

#define VECTOR_PRECISION double;

constexpr unsigned int WIDTH = 70, HEIGHT = 70;
#define MAX_STEPS 40
#define THRESHOLD 0.01
#define MAX_DISTANCE 10

std::vector<std::thread> g_activeThreads {};

// Vectors
struct Vec2 {
	using t_P = VECTOR_PRECISION;
	t_P x;
	t_P y;
	Vec2(t_P xIn, t_P yIn) : x(xIn), y(yIn) {}

	template <typename T> Vec2 operator+(T n) {return Vec2{x+n, y+n};}
	Vec2 operator+(Vec2 n) {return Vec2{x+n.x, y+n.y};}
	template <typename T> Vec2 operator*(T n) {return Vec2{n*x, n*y};}
	template <typename T> Vec2 operator-(T n) {return Vec2{x-n, y-n};}
	Vec2 operator-(Vec2 n) {return *this+(n*-1);}
	template <typename T> Vec2 operator/(T n) {return *this*(1.0/n);}

protected:
	t_P abs_(t_P n) {return n<0 ? n*-1 : n;}

public:
	Vec2 abs() {return Vec2{abs_(x), abs_(y)};}
	double length() {return sqrt(pow(x, 2) + pow(y, 2));}
	auto normalize() {return *this/(this->length());}
};

struct Vec3 : public Vec2 {
	t_P z;
	Vec3(t_P xIn, t_P yIn, t_P zIn) : z(zIn), Vec2(xIn, yIn) {}

	template <typename T> Vec3 operator+(T n) {return Vec3{x+n, y+n, z+n};}
	Vec3 operator+(Vec3 n) {return Vec3{x+n.x, y+n.y, z+n.z};}
	template <typename T> Vec3 operator*(T n) {return Vec3{n*x, n*y, n*z};}
	template <typename T> Vec3 operator-(T n) {return Vec3{x-n, y-n, z-n};}
	Vec3 operator-(Vec3 n) {return *this+(n*-1);}
	template <typename T> Vec3 operator/(T n) {return *this*(1.0/n);}

	Vec3 abs() {return Vec3{abs_(x), abs_(y), abs_(z)};}
	double length() {return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));}
	auto normalize() {return *this/(this->length());}
};

struct Vec4 : public Vec3 {
	t_P w;
	Vec4(t_P xIn, t_P yIn, t_P zIn, t_P wIn) : w(wIn), Vec3(xIn, yIn, zIn) {}

	template <typename T> Vec4 operator+(T n) {return Vec4{x+n, y+n, z+n, w+n};}
	Vec4 operator+(Vec4 n) {return Vec4{x+n.x, y+n.y, z+n.z, w+n.w};}
	template <typename T> Vec4 operator*(T n) {return Vec4{n*x, n*y, n*z, n*w};}
	template <typename T> Vec4 operator-(T n) {return Vec4{x-n, y-n, z-n, w-n};}
	Vec4 operator-(Vec4 n) {return *this+(n*-1);}
	template <typename T> Vec4 operator/(T n) {return *this*(1.0/n);}

	Vec4 abs() {return Vec4{abs_(x), abs_(y), abs_(z), abs_(w)};}
	double length() {return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2) + pow(w, 2));}
	auto normalize() {return *this/(this->length());}
};

double dot(Vec3 x, Vec3 y) {
	return x.x * y.x + x.y * y.y + x.z * y.z;
}

Vec3 reflect(Vec3 d, Vec3 n) {
	return d - n*(dot(d*2, n)/pow(n.length(), 2));
}

// Sdf
template <typename T>
T max (T x, T y) {
	return x > y ? x : y;
}

template <typename T>
T min (T x, T y) {
	return x < y ? x : y;
}

// ---

double union_sdf(double f1, double f2) {
    return min(f1, f2);
}

double intersect_sdf(double f1, double f2) {
    return max(f1, f2);
}

double blend_sdf(double f1, double f2, double k) {
    return f1*k + f2*(1.0-k);
}

double smooth_union_sdf(double f1, double f2, double k) {
    double res = exp(-k*f1) + exp(-k*f2);
	return -log(max(0.0001,res)) / k;
}

double subtract_sdf(double f1, double f2) {
	return intersect_sdf(f1, -f2);
}

// ---

double sphere_sdf(Vec3 samplingPoint, double radius, Vec3 center) {
	return (samplingPoint - center).length() - radius;
}

double cube_sdf(Vec3 samplingPoint, double sideLength, Vec3 center) {
	Vec3 temp = (samplingPoint - center).abs() - sideLength/2.0;
    return max(max(temp.x, temp.y), temp.z);

	return sideLength;
}

double torus_sdf(Vec3 samplingPoint, Vec2 t, Vec3 center) {
	auto temp = samplingPoint-center;
	Vec2 q = Vec2{Vec2{temp.x, temp.z}.length()-t.x, temp.y};
  	return q.length()-t.y;
}

double scene_sdf(Vec3 samplingPoint, const double time) {
	auto sphere = sphere_sdf(samplingPoint, 1.0, Vec3{1.0, 1.0, 4.0});
	auto cube = cube_sdf(samplingPoint, 2.0, Vec3{0.0, 0.0, 5.0});
	// auto result = subtract_sdf(cube, sphere);
	auto result = torus_sdf(samplingPoint, Vec2{1.0, 0.5}, Vec3{0.0, 1.5, 5.0});
	// auto result = sphere_sdf(samplingPoint, 1.0, Vec3{0.0, 0.0, 4.0});
	return result;
	// return -1;
}

// Ray march in a given direction and get the color of the ray
Vec3 get_normal(Vec3 p, double(*sdf)(Vec3, const double), const double time) {
    return (Vec3(
        sdf(Vec3(p.x + THRESHOLD, p.y, p.z), time) - sdf(Vec3(p.x - THRESHOLD, p.y, p.z), time),
        sdf(Vec3(p.x, p.y + THRESHOLD, p.z), time) - sdf(Vec3(p.x, p.y - THRESHOLD, p.z), time),
        sdf(Vec3(p.x, p.y, p.z + THRESHOLD), time) - sdf(Vec3(p.x, p.y, p.z - THRESHOLD), time)
    )).normalize();
}

Vec3 get_col(Vec3 color, Vec3 currentPos, Vec3 lightPos, Vec3 cameraPos, Vec3 normal) {
    Vec3 lightDir = (lightPos - currentPos).normalize(); 
    Vec3 cameraDir = (cameraPos - currentPos).normalize();
    
    const float ambient = 0.3;
    
    float diffuse = max(dot(lightDir, normal), 0.0);
    
    float iRoughness = 5.0; float specStrength = 1.0;
    float specular = max(pow(dot(reflect(lightDir, normal), cameraDir * -1), iRoughness), 0.0) * specStrength;
    
    return color * (diffuse + ambient) + specular;
}

Vec3 ray_march(Vec3 dir, double(*sdf)(Vec3, const double), double time) {
	Vec3 currentPos = Vec3{0.0, 0.0, 0.0};
	Vec3 col = Vec3{0.0, 0.0, 0.0};
	// std::cout << time << ", ";

    for (unsigned int i=0; i<MAX_STEPS; i++) {
        float dist = sdf(currentPos, time);
		if (dist >= MAX_DISTANCE) break;
        if (dist <= THRESHOLD) {
            // col = Vec3{0.0, 0.0, 0.0};
			col = get_col(Vec3{0.0, 0.0, 1.0}, currentPos, Vec3{sin(time/1000)*10.0, sin(time/1000)*10.0, sin(time/2000)*10.0}, Vec3{0.0, 0.0, 0.0}, get_normal(currentPos, sdf, time));
            break;
        }
        currentPos = currentPos + (dir * dist);
    }
	
    return col;
}

Vec3 get_ray_dir(Vec2 uv) {
	// FIXME: Check aspect ratio is working correctly
	const double aspectRatio = WIDTH/HEIGHT;
	return Vec3{((uv.x/(WIDTH-1))-0.5)*aspectRatio, (uv.y/(HEIGHT-1))-0.5, 1.0}.normalize();
}

// Render
char char_from_brightness(double brightness) {
	// return brightness <= 0.125 ? '@' : brightness <= 0.25 ? '#' : brightness <= 0.375 ? '/' : brightness <= 0.5 ? '*' : brightness <= 0.625 ? ';' : brightness <= 0.75 ? ',' : brightness <= 0.875 ? '.' : ' ';
	return brightness <= 0.125 ? ' ' : brightness <= 0.25 ? '.' : brightness <= 0.375 ? ',' : brightness <= 0.5 ? ';' : brightness <= 0.625 ? '*' : brightness <= 0.75 ? '/' : brightness <= 0.875 ? '#' : '@';
}

std::string draw(double(*sdf)(Vec3, const double), Vec3 samplingPoint) {
	auto time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

	std::string out = "";
	for (unsigned int i=0; i<HEIGHT; i++) {
		for (unsigned int j=0; j<WIDTH; j++) {
			Vec3 color = ray_march(get_ray_dir(Vec2{ (Vec2::t_P)j, (Vec2::t_P)i }), sdf, time);
			double brightness = (color.x + color.y + color.z) / 3.0;
			out += char_from_brightness(brightness);
		}
	}
	return out;
}

// Run function on thread every x time intervals
template<class Func, class ...Args>
void set_interval(int msec, Func func, Args &&...args){
    auto endless = [=](Args &&...args){
        while(true){
            func(args...);
            std::this_thread::sleep_for(std::chrono::milliseconds(msec));
        }
    };

    std::thread thread(endless, args...);

    g_activeThreads.push_back(move(thread));
}

void join_threads() {
	for (int i=0; i<g_activeThreads.size(); i++) {
		g_activeThreads[i].join();
	}
}

// Gameloop
bool gameloop() {
	// std::cout << "\33[2J";
	std::cout << "\33[" << HEIGHT << "A"; // Return to top of screen
	
	std::string raw = draw(scene_sdf, Vec3{0.0, 0.0, 0.0});
	std::string formatted = "";
	for (int i=0; i<raw.length(); i++) {
		formatted += raw[i];
		formatted += " ";
		if ((i+1)%WIDTH==0) formatted += "\n";
	}
	std::cout << formatted;

	return false;
}

// Main
int main() {
	std::cout << "\33[?25l"; // Hide cursor
	std::cout << "\33[3J"; // Clear scrollback
	set_interval(1000.0/60, gameloop);

	join_threads();
	std::cout << "\33[?25h"; // Show cursor

	return 0;
}

// TODO: Add rotation
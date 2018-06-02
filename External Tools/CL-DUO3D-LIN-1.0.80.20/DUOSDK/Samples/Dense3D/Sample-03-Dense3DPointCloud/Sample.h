///////////////////////////////////////////////////////////////////////////////////
// This code sample demonstrates the use of DUO SDK in your own applications
// For updates and file downloads go to: http://duo3d.com/
// Copyright 2014-2016 (c) Code Laboratories, Inc.  All rights reserved.
///////////////////////////////////////////////////////////////////////////////////
#ifndef SAMPLE_H
#define SAMPLE_H

#if defined(WIN32)
#include "GL/glut.h"
#elif defined(__linux__)
#include <GL/glut.h>
#elif defined(__APPLE__)
#include <GLUT/glut.h>
#endif

#include <queue>
#include <mutex>
#include <thread>

// Include DUO API header file
#include <DUOLib.h>
// Include Dense3D API header file
#include <Dense3DMT.h>

#include <opencv2/opencv.hpp>
using namespace cv;

namespace cl 
{
	class Vec
	{
	public:
		union
		{
			struct { double x, y, z; };
			double v_[3];
		};
	public:
		Vec() : x(0.0), y(0.0), z(0.0) {}
		Vec(double X, double Y, double Z) : x(X), y(Y), z(Z) {}
		Vec(cv::Vec3b v) : x(v.val[0]), y(v.val[1]), z(v.val[2]) {}

		Vec& operator=(const Vec& v)
		{
			x = v.x;   y = v.y;   z = v.z;
			return *this;
		}
		double operator[](int i) const { return v_[i]; }
		double& operator[](int i) {	return v_[i]; }
		operator const double*() const { return v_;	}
		operator double*() { return v_; }

		friend Vec operator+(const Vec &a, const Vec &b)
		{
			return Vec(a.x + b.x, a.y + b.y, a.z + b.z);
		}
		friend Vec operator-(const Vec &a, const Vec &b)
		{
			return Vec(a.x - b.x, a.y - b.y, a.z - b.z);
		}
		friend Vec operator-(const Vec &a)
		{
			return Vec(-a.x, -a.y, -a.z);
		}
		friend Vec operator*(const Vec &a, double k)
		{
			return Vec(a.x * k, a.y * k, a.z * k);
		}
		friend Vec operator*(double k, const Vec &a)
		{
			return a * k;
		}
		friend Vec operator/(const Vec &a, double k)
		{
			return Vec(a.x / k, a.y / k, a.z / k);
		}
		friend bool operator!=(const Vec &a, const Vec &b)
		{
			return !(a==b);
		}
		friend bool operator==(const Vec &a, const Vec &b)
		{
			const double epsilon = 1.0E-10f;
			return (a-b).squaredNorm() < epsilon;
		}
		Vec& operator+=(const Vec &a)
		{
			x += a.x; y += a.y; z += a.z;
			return *this;
		}
		Vec& operator-=(const Vec &a)
		{
			x -= a.x; y -= a.y; z -= a.z;
			return *this;
		}
		Vec& operator*=(double k)
		{
			x *= k; y *= k; z *= k;
			return *this;
		}
		Vec& operator/=(double k)
		{
			x /= k; y /= k; z /= k;
			return *this;
		}
		friend double operator*(const Vec &a, const Vec &b)
		{
			return a.x * b.x + a.y * b.y + a.z * b.z;
		}
		friend Vec operator^(const Vec &a, const Vec &b)
		{
			return cross(a, b);
		}
		friend Vec cross(const Vec &a, const Vec &b)
		{
			return Vec(a.y * b.z - a.z * b.y,
				a.z * b.x - a.x * b.z,
				a.x * b.y - a.y * b.x);
		}
		double squaredNorm() const { return x * x + y * y + z * z; }
		double norm() const { return sqrt(x * x + y * y + z * z); }
		double normalize()
		{
			const double n = norm();
			*this /= n;
			return n;
		}
		void rotate(double angle, Vec axis)
		{
			double rad = angle * (3.14159265358979323846 / 180.0);
			double cc = cos(rad);
			double ss = sin(rad);
			double a = axis.x * axis.x + (1 - axis.x * axis.x) * cc;
			double b = axis.x * axis.y * (1 - cc) - axis.z * ss;
			double c = axis.x * axis.z * (1 - cc) + axis.y * ss;
			double d = axis.x * axis.y * (1 - cc) + axis.z * ss;
			double e = axis.y * axis.y + (1 - axis.y * axis.y) * cc;
			double f = axis.y * axis.z * (1 - cc) - axis.x * ss;
			double g = axis.x * axis.z * (1 - cc) - axis.y * ss;
			double h = axis.y * axis.z * (1 - cc) + axis.x * ss;
			double i = axis.z * axis.z + (1 - axis.z * axis.z) * cc;
			double nx = x * a + y * b + z * c;
			double ny = x * d + y * e + z * f;
			double nz = x * g + y * h + z * i;
			x = nx;
			y = ny;
			z = nz;
		}
		static double angle(Vec a, Vec c, Vec b)
		{
			double s = acos((a-c) * (c-b) / ((a-c).norm() * (c-b).norm()));
			return s * (180.0 / 3.14159265358979323846);
		}
	};

	class TrackballCamera
	{
		Vec _position, _lookAt, _forward, _up, _left;
		double _angleX;
	public:
		TrackballCamera() {}
		TrackballCamera(Vec position, Vec lookat)
		{
			_position = position;
			_lookAt = lookat;
			_angleX = 0.0;
			update();
		}
		void update()
		{
			_forward = _lookAt - _position;
			_left = Vec(_forward.z, 0, -_forward.x);
			_up = cross(_left, _forward);
			_forward.normalize();
			_left.normalize();
			_up.normalize();
		}
		void show()
		{
			gluLookAt(_position.x, _position.y, _position.z,
				_lookAt.x, _lookAt.y, _lookAt.z,
				0.0, 1.0, 0.0);
		}
		void rotate(double pos, Vec v)
		{
			Vec prevPos = _position;
			translate(-_lookAt);
			_position.rotate(pos / 500.0, v);
			translate(_lookAt);
			updateAngleX();
			if(_angleX < 5 || _angleX > 175)
			{
				_position = prevPos;
				updateAngleX();
			}
		}
		void translate(Vec v)
		{
			_position += v;
		}
		void translateLookAt(Vec v)
		{
			_lookAt += v;
		}
		void translateAll(Vec v)
		{
			translate(v);
			translateLookAt(v);
		}
		void zoom(double z)
		{
			double dist = (_position - _lookAt).norm();
			if (dist - z > z)
				translate(_forward * z);
		}
		Vec getPosition() { return _position; }
		Vec getPositionFromLookAt() { return _position - _lookAt; }
		Vec getLookAt() { return _lookAt; }
		Vec getForward() { return _forward; }
		Vec getUp() { return _up; }
		Vec getLeft() { return _left; }
		void setPosition(Vec p) { _position = p; updateAngleX(); }
		void setLookAt(Vec p) { _lookAt = p; updateAngleX(); }
	private:
		void updateAngleX()
		{
			_angleX = Vec::angle(Vec(_position.x, _position.y + 1, _position.z), _position, _lookAt);
		}
	};

	struct PointXYZRGB
	{
		double x, y, z;
		double r, g, b;
		PointXYZRGB() : x(0), y(0), z(0), r(0), g(0), b(0) {}
		PointXYZRGB(double x_, double y_, double z_) : x(x_), y(y_), z(z_), r(0), g(0), b(0) {}
		PointXYZRGB(double x_, double y_, double z_, double r_, double g_, double b_) : x(x_), y(y_), z(z_), r(r_), g(g_), b(b_) {}
		PointXYZRGB(Vec v, double r_, double g_, double b_) : x(v.x), y(v.y), z(v.z), r(r_), g(g_), b(b_) {}
		PointXYZRGB(Vec v, double c) : x(v.x), y(v.y), z(v.z), r(c), g(c), b(c) {}
	};

	class CloudViewer
	{
		std::mutex _cloudMx;
		std::vector<PointXYZRGB> _cloud;
		bool _rotate, _translate, _zoom;
		int _startx, _starty;
		TrackballCamera _camera;
		GLfloat _pointSize;
		double _fovX, _fovY;
		char _title[256];
		std::function<void()> _idle;
		std::function<void()> _exit;
	public:
		static CloudViewer *_this;
	public:
		CloudViewer(const char *title = "DUO 3D Viewer")
		{
			strcpy(_title, title);
			_this = this;
			_camera = TrackballCamera(Vec(0.0, 0.0, 0.5), Vec(0.0, 0.0, 0.0));
			_pointSize = 3;
			_translate = _rotate = _zoom = false;
		}
        virtual ~CloudViewer() {}
        void addData(const Mat1b &image, const Mat3f &depth)
		{
			_cloudMx.lock();
			_cloud.clear();
			for(int y = 0; y < image.rows; y++)
				for(int x = 0; x < image.cols; x++)
				{
					Point p = Point(x, y);
					double c = image.at<uchar>(p) / 255.0;
					if(c == 0) continue;
					Vec v(depth.at<Vec3f>(p)[0], depth.at<Vec3f>(p)[1], depth.at<Vec3f>(p)[2]);
					if(v.z < 10000)
					{
						v /= 1000.0;
						_cloud.push_back(PointXYZRGB(v, c));
					}
				}
			_cloudMx.unlock();
		}
		void setFov(double fovX, double fovY)
		{
 			_fovX = fovX / 180.0 * 3.14159265358979323846;
 			_fovY = fovY / 180.0 * 3.14159265358979323846;
		}
		void onIdle(std::function<void()> idle)
		{
			_idle = idle;
		}
		void onExit(std::function<void()> exit)
		{
			_exit = exit;
		}
		void run()
		{
			init();
			atexit([](){ _this->exit(); });
			glutMainLoop();
		}
	private:
		void init()
		{
			int argc = 1;
			char *argv[1] = { _title };
			glutInit(&argc, argv);
			glutInitWindowSize(640, 480);
			glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
			glutCreateWindow(_title);
			glEnable(GL_DEPTH_TEST);
			glMatrixMode(GL_PROJECTION);
			gluPerspective(75.0, 1.0, 0.001, 20000.0);
			glMatrixMode(GL_MODELVIEW);
			gluLookAt(0.0, 0.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0);
			glShadeModel(GL_SMOOTH);
			glDepthFunc(GL_LEQUAL);
			glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
			glutDisplayFunc([](){ if(glutGetWindow()) _this->draw(); });
			glutMouseFunc([](int button, int state, int x, int y){ if(glutGetWindow()) _this->mouse(button, state, x, y); });
			glutKeyboardFunc([](unsigned char c, int x, int y){ if(glutGetWindow()) _this->key(c, x, y); });
			glutMotionFunc([](int x, int y){ if(glutGetWindow()) _this->motion(x, y); });
			glutReshapeFunc([](int width, int height){ if(glutGetWindow()) _this->reshape(width, height); });
			glutIdleFunc([](){ _this->idle(); });
			glClearDepth(1.0);
			glEnable(GL_DEPTH_TEST);
			glDepthFunc(GL_LEQUAL);
			glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
		}
		void idle()
		{
			if(_idle)
			{
				_idle();
				if(glutGetWindow()) 
					glutPostRedisplay();
			}
			if(glutGetWindow() == 0)
				_exit();
		}
		void exit()
		{
			if(_exit) 
				_exit(); 
		}
		void draw()
		{
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glLoadIdentity();
			glPushMatrix();
			_camera.update();
			_camera.show();
			glEnable(GL_BLEND);
			glDisable(GL_LIGHTING);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glClearColor(0.12f, 0.12f, 0.12f, 1.0f);
			drawPointCloud();
			drawOrigin(_camera.getLookAt());
			drawFOV();
			glColor4d(0, 0, 1, 0.75);
			glutSolidCube(0.05);
			glPopMatrix();
			glutSwapBuffers();
		}
		void mouse(int button, int state, int x, int y)
		{
			if (button == GLUT_LEFT_BUTTON)
			{
				if (state == GLUT_DOWN)
				{
					_rotate = true;
					_startx = x;
					_starty = y;
				}
				if (state == GLUT_UP)
					_rotate = false;
			}
			else if (button == GLUT_RIGHT_BUTTON)
			{
				if (state == GLUT_DOWN)
				{
					_translate = true;
					_startx = x;
					_starty = y;
				}
				if (state == GLUT_UP)
					_translate = false;
			}
			else if (button == GLUT_MIDDLE_BUTTON)
			{
				if (state == GLUT_DOWN)
				{
					_zoom = true;
					_startx = x;
					_starty = y;
				}
				if (state == GLUT_UP)
					_zoom = false;
			}
			// Wheel reports as button 3(scroll up) and button 4(scroll down)
			else if ((button == 3) || (button == 4))
			{
				if (state == GLUT_UP)
					return;
				if (button == 3)
					_camera.zoom(0.1);
				else
					_camera.zoom(-0.1);
			}
		}
		void key(unsigned char c, int x, int y)
		{
			switch (c)
			{
				case '-':
					if (_pointSize > 0)
						_pointSize--;
				break;
				case '+':
					_pointSize++;
				break;
				case ' ':
					_camera.setPosition(Vec(0.0, 0.0, 0.5));
					_camera.setLookAt(Vec(0.0, 0.0, 0.0));
				break;
				default:
				break;
			}
			glutPostRedisplay();
		}
		void motion(int x, int y)
		{
			double sensitivity = 100.0;
			if (_translate)
			{
				Vec left = _camera.getLeft();
				Vec up = _camera.getUp();
				_camera.translateAll(left * (x - _startx) / sensitivity);
				_camera.translateAll(up * -(y - _starty) / sensitivity);
				_startx = x;
				_starty = y;
			}
			if (_zoom)
			{
				_camera.zoom((y - _starty) / 5.0);
				_starty = y;
			}
			if (_rotate)
			{
				double rot = y - _starty;
				Vec tmp = Vec(-_camera.getPositionFromLookAt().z, 0, _camera.getPositionFromLookAt().x);
				tmp.normalize();
				_camera.rotate(rot * sensitivity, tmp);
				rot = x - _startx;
				_camera.rotate(-rot * sensitivity, Vec(0.0, 1.0, 0.0));
				_startx = x;
				_starty = y;
			}
			glutPostRedisplay();
		}
		void reshape(int width, int height)
		{
			glViewport(0, 0, width, height);
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			gluPerspective(75.0, (double)width / (double)height, 0.001, 20000.0);
			glMatrixMode(GL_MODELVIEW);
		}
		void drawPointCloud()
		{
			glPointSize(_pointSize);
			glBegin(GL_POINTS);
			_cloudMx.lock();
			for (auto it = _cloud.begin(); it != _cloud.end(); ++it)
			{
				glColor4d(it->r, it->g, it->b, 0.75);
				glVertex3d(it->x, -it->y, -it->z);
			}
			_cloudMx.unlock();
			glEnd();
		}
		void drawOrigin(Vec pos)
		{
			double length = 0.1;
			glBegin(GL_LINES);
			glColor3d(1, 0, 0);
			glVertex3d(pos.x, pos.y, pos.z);
			glVertex3d(pos.x + length, pos.y, pos.z);
			glColor3d(0, 1, 0);
			glVertex3d(pos.x, pos.y, pos.z);
			glVertex3d(pos.x, pos.y + length, pos.z);
			glColor3d(0.25f, 0.25f, 1);
			glVertex3d(pos.x, pos.y, pos.z);
			glVertex3d(pos.x, pos.y, pos.z + length);
			glEnd();
		}
		void drawFOV()
		{
			double z = 0.1;
			double x = tan(_fovX / 2.0) * z;
			double y = tan(_fovY / 2.0) * z;
			glBegin(GL_LINES);
			glColor3d(0.2, 0.2, 0.2);
			glVertex3d(0.0, 0.0, 0.0);	glVertex3d(-x, -y, -z);
			glVertex3d(0.0, 0.0, 0.0);	glVertex3d(x, -y, -z);
			glVertex3d(0.0, 0.0, 0.0);	glVertex3d(-x, y, -z);
			glVertex3d(0.0, 0.0, 0.0);	glVertex3d(x, y, -z);
			glVertex3d(-x, -y, -z);		glVertex3d(x, -y, -z);
			glVertex3d(x, -y, -z);		glVertex3d(x, y, -z);
			glVertex3d(x, y, -z);		glVertex3d(-x, y, -z);
			glVertex3d(-x, y, -z);		glVertex3d(-x, -y, -z);
			glEnd();
		}
	};

	CloudViewer *CloudViewer::_this = NULL;

	typedef struct
	{
	    Mat leftImg;
	    Mat depth;
	}D3DFrame;

	class Dense3DFrameQueue 
	{
		std::mutex mutex;
		std::queue<D3DFrame> queue;
	public:
		void push(D3DFrame frame)
		{
			mutex.lock();
			queue.push(frame);
			mutex.unlock();
		}
		bool pop(D3DFrame &frame)
		{
			mutex.lock();
			if(queue.size() == 0)
			{
				mutex.unlock();
				return false;
			}
			frame = queue.front();
			queue.pop();
			mutex.unlock();
			return true;
		}
	};
}

#endif // SAMPLE_H

#ifndef POINT3D_H
#define POINT3D_H

//!\brief Class point3d has got only 3 points:x,y,z. It is used in calculation of vectors of trajectories and distances of the objects
//!point3d class point3d constructor
//!~point3d class point3d deconstructor
//!
class point3d
{
     public:
       double x;
	double y;
	double z;

	point3d(double f, double g, double h);
	point3d();
        virtual ~point3d();

};

#endif // POINT3D_H

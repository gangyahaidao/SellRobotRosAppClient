#include "myvector4d.h"

MyVector4D::MyVector4D()
{
    x = y = z = w = 0;
}

MyVector4D::MyVector4D(float _x, float _y, float _z, float _w)
{
    x = _x;
    y = _y;
    z = _z;
    w = _w;
}

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                       This file is part of 'project'                       *
 ******************************************************************************/
auto u = -n*x*pow(r/r1, n)*(c*cos(m*atan2(y, x)) + s*sin(m*atan2(y, x)))*assoc_legendre(n, m, z/r1)/(pow(x, 2) + pow(y, 2) + pow(z, 2)) - x*z*pow(r/r1, n)*(c*cos(m*atan2(y, x)) + s*sin(m*atan2(y, x)))*(n*z*assoc_legendre(n, m, z/r1)/r1 - (m + n)*assoc_legendre(n - 1, m, z/r1))/((pow(z, 2)/(pow(x, 2) + pow(y, 2) + pow(z, 2)) - 1)*pow(pow(x, 2) + pow(y, 2) + pow(z, 2), 3.0/2.0)) + pow(r/r1, n)*(c*m*y*sin(m*atan2(y, x))/(pow(x, 2) + pow(y, 2)) - m*s*y*cos(m*atan2(y, x))/(pow(x, 2) + pow(y, 2)))*assoc_legendre(n, m, z/r1);
auto v = -n*y*pow(r/r1, n)*(c*cos(m*atan2(y, x)) + s*sin(m*atan2(y, x)))*assoc_legendre(n, m, z/r1)/(pow(x, 2) + pow(y, 2) + pow(z, 2)) - y*z*pow(r/r1, n)*(c*cos(m*atan2(y, x)) + s*sin(m*atan2(y, x)))*(n*z*assoc_legendre(n, m, z/r1)/r1 - (m + n)*assoc_legendre(n - 1, m, z/r1))/((pow(z, 2)/(pow(x, 2) + pow(y, 2) + pow(z, 2)) - 1)*pow(pow(x, 2) + pow(y, 2) + pow(z, 2), 3.0/2.0)) + pow(r/r1, n)*(-c*m*x*sin(m*atan2(y, x))/(pow(x, 2) + pow(y, 2)) + m*s*x*cos(m*atan2(y, x))/(pow(x, 2) + pow(y, 2)))*assoc_legendre(n, m, z/r1);
auto w = -n*z*pow(r/r1, n)*(c*cos(m*atan2(y, x)) + s*sin(m*atan2(y, x)))*assoc_legendre(n, m, z/r1)/(pow(x, 2) + pow(y, 2) + pow(z, 2)) + pow(r/r1, n)*(c*cos(m*atan2(y, x)) + s*sin(m*atan2(y, x)))*(-pow(z, 2)/pow(pow(x, 2) + pow(y, 2) + pow(z, 2), 3.0/2.0) + pow(pow(x, 2) + pow(y, 2) + pow(z, 2), -1.0/2.0))*(n*z*assoc_legendre(n, m, z/r1)/r1 - (m + n)*assoc_legendre(n - 1, m, z/r1))/(pow(z, 2)/(pow(x, 2) + pow(y, 2) + pow(z, 2)) - 1);
//>>> g
//(r/sqrt(x**2 + y**2 + z**2))**n*(c*cos(m*atan2(y, x)) + s*sin(m*atan2(y, x)))*assoc_legendre(n, m, z/sqrt(x**2 + y**2 + z**2))
//>>> p
//atan2(z, sqrt(x**2 + y**2))
//>>> l
//atan2(y, x)
//>>> [(c_name, c_code), (h_name, c_header)] = codegen(('sympyCalculateGeoPotentialDerivitive', [Eq(u0, diff(g, x0)) for u0, x0 in zip((u, v, w), (x, y, z))]), 'C99')
//>>> print(c_code.replace('sqrt(x**2 + y**2 + z**2)', 'r1').replace('(*u)', 'auto u').replace('(*v)', 'auto v').replace('(*w)', 'auto w').replace('sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2))', 'r1').replace('   auto', 'auto'))

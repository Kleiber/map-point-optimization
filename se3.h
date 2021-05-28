#ifndef SE3_H
#define SE3_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 4> Matrix34d;

class SE3{
    public:
        SE3();
        SE3(cv::Mat pT);
        SE3(Eigen::Matrix3d pRotation, Eigen::Vector3d ptraslation);

        Eigen::Vector3d getTraslation();
        Eigen::Matrix3d getRotation();
        Matrix34d getTransformation();

        Eigen::Matrix3d skewSO3(const Eigen::Vector3d &pw);
        Eigen::Matrix3d expSO3(const Eigen::Vector3d &pw);
        Eigen::Vector3d logSO3(const Eigen::Matrix3d &pRotation);
        Matrix34d expSE3(const Vector6d &ptwist);
        Vector6d logSE3(const Matrix34d &pT);

        void Euler(double prx, double pry, double prz, double ptx, double pty, double ptz);
        void Quaternion(double pqw, double pqx, double pqy, double pqz, double ptx, double pty, double ptz);
        void Lie(double pwx, double pwy, double pwz, double ptx, double pty, double ptz);

        Eigen::Vector3d traslate(const Eigen::Vector3d &pPoint);
        Eigen::Vector3d rotate(const Eigen::Vector3d &pPoint);
        Eigen::Vector3d transform(const Eigen::Vector3d &pPoint);

        SE3 inverse();

        SE3 operator*(const SE3 &T);
        Eigen::Vector3d operator*(const Eigen::Vector3d &pPoint);

        Eigen::Vector3d t;
        Eigen::Matrix3d R;
};

#endif // SE3_H


/*
 * // A simple quickref for Eigen. Add anything that's missing.
// Main author: Keir Mierle

#include <Eigen/Dense>

Matrix<double, 3, 3> A;               // Fixed rows and cols. Same as Matrix3d.
Matrix<double, 3, Dynamic> B;         // Fixed rows, dynamic cols.
Matrix<double, Dynamic, Dynamic> C;   // Full dynamic. Same as MatrixXd.
Matrix<double, 3, 3, RowMajor> E;     // Row major; default is column-major.
Matrix3d P, Q, R;                     // 3x3 double matrix.
Vector3d x, y, z;                     // 3x1 double matrix.
RowVector3d a, b, c;                  // 1x3 double matrix.
VectorXd v;                           // Dynamic column vector of doubles
double s;

// Basic usage
// Eigen          // Matlab           // comments
x.size()          // length(x)        // vector size
C.rows()          // size(C,1)        // number of rows
C.cols()          // size(C,2)        // number of columns
x(i)              // x(i+1)           // Matlab is 1-based
C(i,j)            // C(i+1,j+1)       //

A.resize(4, 4);   // Runtime error if assertions are on.
B.resize(4, 9);   // Runtime error if assertions are on.
A.resize(3, 3);   // Ok; size didn't change.
B.resize(3, 9);   // Ok; only dynamic cols changed.

A << 1, 2, 3,     // Initialize A. The elements can also be
     4, 5, 6,     // matrices, which are stacked along cols
     7, 8, 9;     // and then the rows are stacked.
B << A, A, A;     // B is three horizontally stacked A's.
A.fill(10);       // Fill A with all 10's.

// Eigen                            // Matlab
MatrixXd::Identity(rows,cols)       // eye(rows,cols)
C.setIdentity(rows,cols)            // C = eye(rows,cols)
MatrixXd::Zero(rows,cols)           // zeros(rows,cols)
C.setZero(rows,cols)                // C = ones(rows,cols)
MatrixXd::Ones(rows,cols)           // ones(rows,cols)
C.setOnes(rows,cols)                // C = ones(rows,cols)
MatrixXd::Random(rows,cols)         // rand(rows,cols)*2-1        // MatrixXd::Random returns uniform random numbers in (-1, 1).
C.setRandom(rows,cols)              // C = rand(rows,cols)*2-1
VectorXd::LinSpaced(size,low,high)   // linspace(low,high,size)'
v.setLinSpaced(size,low,high)        // v = linspace(low,high,size)'


// Matrix slicing and blocks. All expressions listed here are read/write.
// Templated size versions are faster. Note that Matlab is 1-based (a size N
// vector is x(1)...x(N)).
// Eigen                           // Matlab
x.head(n)                          // x(1:n)
x.head<n>()                        // x(1:n)
x.tail(n)                          // x(end - n + 1: end)
x.tail<n>()                        // x(end - n + 1: end)
x.segment(i, n)                    // x(i+1 : i+n)
x.segment<n>(i)                    // x(i+1 : i+n)
P.block(i, j, rows, cols)          // P(i+1 : i+rows, j+1 : j+cols)
P.block<rows, cols>(i, j)          // P(i+1 : i+rows, j+1 : j+cols)
P.row(i)                           // P(i+1, :)
P.col(j)                           // P(:, j+1)
P.leftCols<cols>()                 // P(:, 1:cols)
P.leftCols(cols)                   // P(:, 1:cols)
P.middleCols<cols>(j)              // P(:, j+1:j+cols)
P.middleCols(j, cols)              // P(:, j+1:j+cols)
P.rightCols<cols>()                // P(:, end-cols+1:end)
P.rightCols(cols)                  // P(:, end-cols+1:end)
P.topRows<rows>()                  // P(1:rows, :)
P.topRows(rows)                    // P(1:rows, :)
P.middleRows<rows>(i)              // P(i+1:i+rows, :)
P.middleRows(i, rows)              // P(i+1:i+rows, :)
P.bottomRows<rows>()               // P(end-rows+1:end, :)
P.bottomRows(rows)                 // P(end-rows+1:end, :)
P.topLeftCorner(rows, cols)        // P(1:rows, 1:cols)
P.topRightCorner(rows, cols)       // P(1:rows, end-cols+1:end)
P.bottomLeftCorner(rows, cols)     // P(end-rows+1:end, 1:cols)
P.bottomRightCorner(rows, cols)    // P(end-rows+1:end, end-cols+1:end)
P.topLeftCorner<rows,cols>()       // P(1:rows, 1:cols)
P.topRightCorner<rows,cols>()      // P(1:rows, end-cols+1:end)
P.bottomLeftCorner<rows,cols>()    // P(end-rows+1:end, 1:cols)
P.bottomRightCorner<rows,cols>()   // P(end-rows+1:end, end-cols+1:end)

// Of particular note is Eigen's swap function which is highly optimized.
// Eigen                           // Matlab
R.row(i) = P.col(j);               // R(i, :) = P(:, i)
R.col(j1).swap(mat1.col(j2));      // R(:, [j1 j2]) = R(:, [j2, j1])

// Views, transpose, etc; all read-write except for .adjoint().
// Eigen                           // Matlab
R.adjoint()                        // R'
R.transpose()                      // R.' or conj(R')
R.diagonal()                       // diag(R)
x.asDiagonal()                     // diag(x)
R.transpose().colwise().reverse(); // rot90(R)
R.conjugate()                      // conj(R)

// All the same as Matlab, but matlab doesn't have *= style operators.
// Matrix-vector.  Matrix-matrix.   Matrix-scalar.
y  = M*x;          R  = P*Q;        R  = P*s;
a  = b*M;          R  = P - Q;      R  = s*P;
a *= M;            R  = P + Q;      R  = P/s;
                   R *= Q;          R  = s*P;
                   R += Q;          R *= s;
                   R -= Q;          R /= s;

// Vectorized operations on each element independently
// Eigen                  // Matlab
R = P.cwiseProduct(Q);    // R = P .* Q
R = P.array() * s.array();// R = P .* s
R = P.cwiseQuotient(Q);   // R = P ./ Q
R = P.array() / Q.array();// R = P ./ Q
R = P.array() + s.array();// R = P + s
R = P.array() - s.array();// R = P - s
R.array() += s;           // R = R + s
R.array() -= s;           // R = R - s
R.array() < Q.array();    // R < Q
R.array() <= Q.array();   // R <= Q
R.cwiseInverse();         // 1 ./ P
R.array().inverse();      // 1 ./ P
R.array().sin()           // sin(P)
R.array().cos()           // cos(P)
R.array().pow(s)          // P .^ s
R.array().square()        // P .^ 2
R.array().cube()          // P .^ 3
R.cwiseSqrt()             // sqrt(P)
R.array().sqrt()          // sqrt(P)
R.array().exp()           // exp(P)
R.array().log()           // log(P)
R.cwiseMax(P)             // max(R, P)
R.array().max(P.array())  // max(R, P)
R.cwiseMin(P)             // min(R, P)
R.array().min(P.array())  // min(R, P)
R.cwiseAbs()              // abs(P)
R.array().abs()           // abs(P)
R.cwiseAbs2()             // abs(P.^2)
R.array().abs2()          // abs(P.^2)
(R.array() < s).select(P,Q);  // (R < s ? P : Q)

// Reductions.
int r, c;
// Eigen                  // Matlab
R.minCoeff()              // min(R(:))
R.maxCoeff()              // max(R(:))
s = R.minCoeff(&r, &c)    // [s, i] = min(R(:)); [r, c] = ind2sub(size(R), i);
s = R.maxCoeff(&r, &c)    // [s, i] = max(R(:)); [r, c] = ind2sub(size(R), i);
R.sum()                   // sum(R(:))
R.colwise().sum()         // sum(R)
R.rowwise().sum()         // sum(R, 2) or sum(R')'
R.prod()                  // prod(R(:))
R.colwise().prod()        // prod(R)
R.rowwise().prod()        // prod(R, 2) or prod(R')'
R.trace()                 // trace(R)
R.all()                   // all(R(:))
R.colwise().all()         // all(R)
R.rowwise().all()         // all(R, 2)
R.any()                   // any(R(:))
R.colwise().any()         // any(R)
R.rowwise().any()         // any(R, 2)

// Dot products, norms, etc.
// Eigen                  // Matlab
x.norm()                  // norm(x).    Note that norm(R) doesn't work in Eigen.
x.squaredNorm()           // dot(x, x)   Note the equivalence is not true for complex
x.dot(y)                  // dot(x, y)
x.cross(y)                // cross(x, y) Requires #include <Eigen/Geometry>

//// Type conversion
// Eigen                           // Matlab
A.cast<double>();                  // double(A)
A.cast<double>();                   // single(A)
A.cast<int>();                     // int32(A)
A.real();                          // real(A)
A.imag();                          // imag(A)
// if the original type equals destination type, no work is done

// Note that for most operations Eigen requires all operands to have the same type:
MatrixXf F = MatrixXf::Zero(3,3);
A += F;                // illegal in Eigen. In Matlab A = A+F is allowed
A += F.cast<double>(); // F converted to double and then added (generally, conversion happens on-the-fly)

// Eigen can map existing memory into Eigen matrices.
double array[3];
Vector3d::Map(array).fill(10);            // create a temporary Map over array and sets entries to 10
int data[4] = {1, 2, 3, 4};
Matrix2i mat2x2(data);                    // copies data into mat2x2
Matrix2i::Map(data) = 2*mat2x2;           // overwrite elements of data with 2*mat2x2
MatrixXi::Map(data, 2, 2) += mat2x2;      // adds mat2x2 to elements of data (alternative syntax if size is not know at compile time)

// Solve Ax = b. Result stored in x. Matlab: x = A \ b.
x = A.ldlt().solve(b));  // A sym. p.s.d.    #include <Eigen/Cholesky>
x = A.llt() .solve(b));  // A sym. p.d.      #include <Eigen/Cholesky>
x = A.lu()  .solve(b));  // Stable and fast. #include <Eigen/LU>
x = A.qr()  .solve(b));  // No pivoting.     #include <Eigen/QR>
x = A.svd() .solve(b));  // Stable, slowest. #include <Eigen/SVD>
// .ldlt() -> .matrixL() and .matrixD()
// .llt()  -> .matrixL()
// .lu()   -> .matrixL() and .matrixU()
// .qr()   -> .matrixQ() and .matrixR()
// .svd()  -> .matrixU(), .singularValues(), and .matrixV()

// Eigenvalue problems
// Eigen                          // Matlab
A.eigenvalues();                  // eig(A);
EigenSolver<Matrix3d> eig(A);     // [vec val] = eig(A)
eig.eigenvalues();                // diag(val)
eig.eigenvectors();               // vec
// For self-adjoint matrices use SelfAdjointEigenSolver<>


Multiplication:
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
  Eigen::Vector3d a(0.5, 3, -0.4);

  Eigen::Vector3d Aa = A * a;
  std::cout << "The multiplication of A * a is " << std::endl << Aa << std::endl;


  Eigen::MatrixXd B = Eigen::MatrixXd::Identity(6, 5);
  Eigen::VectorXd b(5);
  b << 1, 4, 6, -2, 0.4;

  Eigen::VectorXd Bb = B * b;
  std::cout << "The multiplication of B * b is " << std::endl << Bb << std::endl;


Transpose and inverse:
    Eigen::MatrixXd A(3, 2);
    A << 1, 2,
         2, 3,
         3, 4;

    Eigen::MatrixXd B = A.transpose();// the transpose of A is a 2x3 matrix
    Eigen::MatrixXd C = (B * A).inverse();// computer the inverse of BA, which is a 2x2 matrix

    Dot product and cross product:
    Eigen::Vector3d v(1, 2, 3);
    Eigen::Vector3d w(0, 1, 2);

    double vDotw = v.dot(w); // dot product of two vectors
    Eigen::Vector3d vCrossw = v.cross(w); // cross product of two vectors


Accessing matrix:

  Eigen::MatrixXd A = Eigen::MatrixXd::Random(7, 9);
  std::cout << "The element at fourth row and 7the column is " << A(3, 6) << std::endl;

  Eigen::MatrixXd B = A.block(1, 2, 3, 3);
  std::cout << "Take sub-matrix whose upper left corner is A(1, 2)" << std::endl << B << std::endl;

  Eigen::VectorXd a = A.col(1); // take the second column of A
  Eigen::VectorXd b = B.row(0); // take the first row of B

  Eigen::VectorXd c = a.head(3);// take the first three elements of a
  Eigen::VectorXd d = b.tail(2);// take the last two elements of b


Quaternion:
  Eigen::Quaterniond q(2, 0, 1, -3);
  std::cout << "This quaternion consists of a scalar " << q.w() << " and a vector " << std::endl << q.vec() << std::endl;
  q.normalize();
  std::cout << "To represent rotation, we need to normalize it such that its length is " << q.norm() << std::endl;


  Eigen::Vector3d v(1, 2, -1);
  Eigen::Quaterniond p;
  p.w() = 0;
  p.vec() = v;

  Eigen::Quaterniond rotatedP = q * p * q.inverse();
  Eigen::Vector3d rotatedV = rotatedP.vec();
  std::cout << "We can now use it to rotate a vector " << std::endl << v << " to " << std::endl << rotatedV << std::endl;


  Eigen::Matrix3d R = q.toRotationMatrix(); // convert a quaternion to a 3x3 rotation matrix
  std::cout << "Compare with the result using an rotation matrix " << std::endl << R * v << std::endl;


  Eigen::Quaterniond a = Eigen::Quterniond::Identity();
  Eigen::Quaterniond b = Eigen::Quterniond::Identity();
  Eigen::Quaterniond c; // Adding two quaternion as two 4x1 vectors is not supported by the EIgen API. That is,
  c = a + b is not allowed. We have to do this in a hard way
  c.w() = a.w() + b.w();
  c.x() = a.x() + b.x();
  c.y() = a.y() + b.y();
  c.z() = a.z() + b.z();
*/

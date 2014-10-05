#ifndef _GEOMETRY_UTILS_H_
#define _GEOMETRY_UTILS_H_

#include <string>
#include <cmath>
#include <limits>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>

namespace geometry_utils
{

  /* *********************************************************************************** */
  namespace math
  {
    template<typename T> inline T cos(T in);
    template<> inline float cos(float in) { return ::cosf(in); }
    template<> inline double cos(double in) { return ::cos(in); }

    template<typename T> inline T acos(T in);
    template<> inline float acos(float in) { return ::acosf(in); }
    template<> inline double acos(double in) { return ::acos(in); }

    template<typename T> inline T sin(T in);
    template<> inline float sin(float in) { return ::sinf(in); }
    template<> inline double sin(double in) { return ::sin(in); }

    template<typename T> inline T asin(T in);
    template<> inline float asin(float in) { return ::asinf(in); }
    template<> inline double asin(double in) { return ::asin(in); }

    template<typename T> inline T tan(T in);
    template<> inline float tan(float in) { return ::tanf(in); }
    template<> inline double tan(double in) { return ::tan(in); }

    template<typename T> inline T atan(T in);
    template<> inline float atan(float in) { return ::atanf(in); }
    template<> inline double atan(double in) { return ::atan(in); }

    template<typename T> inline T fabs(T in);
    template<> inline float fabs(float in) { return ::fabsf(in); }
    template<> inline double fabs(double in) { return ::fabs(in); }

    template<typename T> inline T fmin(T v1, T v2);
    template<> inline float fmin(float v1, float v2) { return ::fminf(v1, v2); }
    template<> inline double fmin(double v1, double v2) { return ::fmin(v1, v2); }

    template<typename T> inline T fmax(T v1, T v2);
    template<> inline float fmax(float v1, float v2) { return ::fmaxf(v1, v2); }
    template<> inline double fmax(double v1, double v2) { return ::fmax(v1, v2); }

    template<typename T> inline T sqrt(T in);
    template<> inline float sqrt(float in) { return ::sqrtf(in); }
    template<> inline double sqrt(double in) { return ::sqrt(in); }

    template<typename T> inline T pow(T in, T exp);
    template<> inline float pow(float in, float exp) { return ::powf(in, exp); }
    template<> inline double pow(double in, double exp) { return ::pow(in, exp); }

    template<typename T> inline T atan2(T y, T x);
    template<> inline float atan2(float y, float x) { return ::atan2f(y, x); }
    template<> inline double atan2(double y, double x) { return ::atan2(y, x); }

    template<typename T> inline T hypot(T x, T y);
    template<> inline float hypot(float x, float y) { return ::hypotf(x, y); }
    template<> inline double hypot(double x, double y) { return ::hypot(x, y); }
  }

  /* *********************************************************************************** */
  template<typename T, size_t N>
  struct VectorNBase
  {
    typedef typename boost::shared_ptr< VectorNBase<T, N> > Ptr;
    typedef typename boost::shared_ptr< const VectorNBase<T, N> > ConstPtr;

    static const size_t length = N;

    boost::array<T, N> data;

    VectorNBase() { data.fill(0); }

    VectorNBase(T val) { data.fill(val); }

    VectorNBase(const VectorNBase& in) : data(in.data) { }

    VectorNBase(const boost::array<T, N>& in) : data(in) { }

    VectorNBase(T (&in)[N])
    {
      for (size_t i = 0; i < N; i++)
        data[i] = in[i];
    }

    inline T& operator()(unsigned int i) { return data[i]; }
    inline const T& operator()(unsigned int i) const { return data[i]; }

    inline T& get(unsigned int i) { return data[i]; }
    inline const T& get(unsigned int i) const { return data[i]; }

    inline VectorNBase& operator=(const VectorNBase& rhs)
    {
      if (this == &rhs)
        return *this;
      data = rhs.data;
      return *this;
    }

    inline VectorNBase operator*(T rhs) const
    {
      T d[N];
      for (size_t i = 0; i < N; i++)
        d[i] = data[i]*rhs;
      return VectorNBase<T, N>(d);
    }

    inline VectorNBase operator+(const VectorNBase& rhs) const
    {
      T d[N];
      for (size_t i = 0; i < N; i++)
        d[i] = data[i] + rhs.data[i];
      return VectorNBase<T, N>(d);
    }

    inline VectorNBase operator-() const
    {
      T d[N];
      for (size_t i = 0; i < N; i++)
        d[i] = -data[i];
      return VectorNBase<T, N>(d);
    }

    inline VectorNBase operator-(const VectorNBase& rhs) const
    {
      T d[N];
      for (size_t i = 0; i < N; i++)
        d[i] = data[i] - rhs.data[i];
      return VectorNBase<T, N>(d);
    }

    inline VectorNBase operator%(const VectorNBase& rhs) const
    {
      T d[N];
      for (size_t i = 0; i < N; i++)
        d[i] = data[i]*rhs.data[i];
      return VectorNBase<T, N>(d);
    }

    inline T operator^(const VectorNBase& rhs) const
    {
      T dot = 0;
      for (size_t i = 0; i < N; i++)
        dot += data[i]*rhs.data[i];
      return dot;
    }

    inline VectorNBase operator/(const VectorNBase& rhs) const
    {
      T d[N];
      for (size_t i = 0; i < N; i++)
        d[i] = data[i]/rhs.data[i];
      return VectorNBase<T, N>(d);
    }

    inline VectorNBase operator+=(const VectorNBase& rhs)
    {
      for (size_t i = 0; i < N; i++)
        data[i] += rhs.data[i];
      return *this;
    }

    inline VectorNBase operator-=(const VectorNBase& rhs)
    {
      for (size_t i = 0; i < N; i++)
        data[i] -= rhs.data[i];
      return *this;
    }

    inline VectorNBase operator%=(const VectorNBase& rhs)
    {
      for (size_t i = 0; i < N; i++)
        data[i] *= rhs.data[i];
      return *this;
    }

    inline VectorNBase operator/=(const VectorNBase& rhs)
    {
      for (size_t i = 0; i < N; i++)
        data[i] /= rhs.data[i];
      return *this;
    }

    inline VectorNBase operator*=(const T& rhs)
    {
      for (size_t i = 0; i < N; i++)
        data[i] *= rhs;
      return *this;
    }

    inline VectorNBase operator/=(const T& rhs)
    {
      for (size_t i = 0; i < N; i++)
        data[i] /= rhs;
      return *this;
    }

    inline bool operator==(const VectorNBase& that) const
    {
      return this->equals(that);
    }

    inline bool operator!=(const VectorNBase& that) const
    {
      return !this->equals(that);
    }

    inline bool equals(const VectorNBase& that, const T ptol = 1e-8) const
    {
      return (*this - that).norm() < ptol;
    }

    inline T norm() const
    {
      return math::sqrt((*this)^(*this));
    }

    inline VectorNBase normalize() const
    {
      return (*this)/norm();
    }

    inline VectorNBase abs() const
    {
      T d[N];
      for (size_t i = 0; i < N; i++)
        d[i] = std::abs(data[i]);
      return VectorNBase<T, N>(d);
    }

    inline void ones()
    {
      data.fill(1);
    }

    inline void zeros()
    {
      data.fill(0);
    }

    inline T dot(const VectorNBase& v) const
    {
      return (*this)^v;
    }

    inline VectorNBase scale(T s) const
    {
      return (*this)*s;
    }

    inline void print(const std::string& prefix = std::string()) const
    {
      if (!prefix.empty())
        std::cout << prefix << std::endl;
      std::cout << (*this) << std::endl;
    }

  };

  template<typename T, size_t N>
  inline VectorNBase<T, N> operator*(const T& lhs, const VectorNBase<T, N>& rhs)
  {
    return rhs*lhs;
  }

  template<typename T, size_t N>
  inline std::ostream& operator<<(std::ostream& out, const VectorNBase<T, N>& m)
  {
    for (size_t i = 0; i < N - 1; i++)
      out << m.data[i] << " ";
    out << m.data[N-1];
    return out;
  }

  template<typename T, size_t N>
  inline T norm(const VectorNBase<T, N>& v)
  {
    return v.norm();
  }

  template<typename T, size_t N>
  inline T dot(const VectorNBase<T, N>& v1, const VectorNBase<T, N>& v2)
  {
    return v1.dot(v2);
  }

  /* *********************************************************************************** */
  template<typename T>
  struct Vector3Base : VectorNBase<T, 3>
  {
    Vector3Base() : VectorNBase<T, 3>() { }
    Vector3Base(T val) : VectorNBase<T, 3>(val) { }
    Vector3Base(const Vector3Base& in) : VectorNBase<T, 3>(in.data) { }
    Vector3Base(const boost::array<T, 3>& in) : VectorNBase<T, 3>(in) { }
    Vector3Base(T (&in)[3]) : VectorNBase<T, 3>(in) { }
    Vector3Base(const VectorNBase<T, 3>& in) : VectorNBase<T, 3>(in) { }

    Vector3Base(T v1, T v2, T v3)
    {
      this->data[0] = v1;
      this->data[1] = v2;
      this->data[2] = v3;
    }

    T x() const { return this->data[0]; }
    T y() const { return this->data[1]; }
    T z() const { return this->data[2]; }

    inline Vector3Base<T> cross(const Vector3Base<T>& v) const
    {
      return Vector3Base<T>(-(*this)(2)*v(1) + (*this)(1)*v(2),
                            (*this)(2)*v(0) - (*this)(0)*v(2),
                            -(*this)(1)*v(0) + (*this)(0)*v(1));
    }
  };

  inline Vector3Base<float> operator*(const float& lhs, const Vector3Base<float>& rhs)
  {
    return Vector3Base<float>(rhs*lhs);
  }

  inline Vector3Base<double> operator*(const double& lhs, const Vector3Base<double>& rhs)
  {
    return Vector3Base<double>(rhs*lhs);
  }

  template<typename T>
  inline VectorNBase<T, 3> cross(const VectorNBase<T, 3>& v1,
                                 const VectorNBase<T, 3>& v2)
  {
    return Vector3Base<T>(v1).cross(v2);
  }

  typedef Vector3Base<float> Vector3f;
  typedef Vector3Base<float> Vec3f;

  typedef Vector3Base<double> Vector3d;
  typedef Vector3Base<double> Vec3d;

  typedef Vector3Base<double> Vector3;
  typedef Vector3Base<double> Vec3;

  /* *********************************************************************************** */
  template<typename T>
  struct Vector2Base : VectorNBase<T, 2>
  {
    Vector2Base() : VectorNBase<T, 2>() { }
    Vector2Base(T val) : VectorNBase<T, 2>(val) { }
    Vector2Base(const Vector2Base& in) : VectorNBase<T, 2>(in.data) { }
    Vector2Base(const boost::array<T, 2>& in) : VectorNBase<T, 2>(in) { }
    Vector2Base(T (&in)[2]) : VectorNBase<T, 2>(in) { }
    Vector2Base(const VectorNBase<T, 2>& in) : VectorNBase<T, 2>(in) { }

    Vector2Base(T v1, T v2)
    {
      this->data[0] = v1;
      this->data[1] = v2;
    }

    T x() const { return this->data[0]; }
    T y() const { return this->data[1]; }
  };

  inline Vector2Base<float> operator*(const float& lhs, const Vector2Base<float>& rhs)
  {
    return Vector2Base<float>(rhs*lhs);
  }

  inline Vector2Base<double> operator*(const double& lhs, const Vector2Base<double>& rhs)
  {
    return Vector2Base<double>(rhs*lhs);
  }

  typedef Vector2Base<float> Vector2f;
  typedef Vector2Base<float> Vec2f;

  typedef Vector2Base<double> Vector2d;
  typedef Vector2Base<double> Vec2d;

  typedef Vector2Base<double> Vector2;
  typedef Vector2Base<double> Vec2;

  /* *********************************************************************************** */
  template<typename T, size_t N, size_t M>
  struct MatrixNxMBase
  {
    typedef typename boost::shared_ptr< MatrixNxMBase<T, N, M> > Ptr;
    typedef typename boost::shared_ptr< const MatrixNxMBase<T, N, M> > ConstPtr;

    static const size_t size = N*M;
    static const size_t nrows = N;
    static const size_t ncols = M;

    boost::array<T, size> data;

    MatrixNxMBase() { data.fill(0); }

    MatrixNxMBase(T val) { data.fill(val); }

    MatrixNxMBase(const MatrixNxMBase& in) : data(in.data) { }

    MatrixNxMBase(const boost::array<T, size>& in) : data(in) { }

    MatrixNxMBase(T (&in)[size])
    {
      for (unsigned int i = 0; i < size; i++)
        data[i] = in[i];
    }

    inline T& operator()(unsigned int i) { return data[i]; }
    inline const T& operator()(unsigned int i) const { return data[i]; }

    inline T& get(unsigned int i) { return data[i]; }
    inline const T& get(unsigned int i) const { return data[i]; }

    inline T& operator()(unsigned int i, unsigned int j)
    {
      return data[ncols*i + j];
    }

    inline const T& operator()(unsigned int i, unsigned int j) const
    {
      return data[ncols*i + j];
    }

    inline T& get(unsigned int i, unsigned int j)
    {
      return data[ncols*i + j];
    }

    inline const T& get(unsigned int i, unsigned int j) const
    {
      return data[ncols*i + j];
    }

    inline std::ostream& operator<<(std::ostream& out)
    {
      out << data;
      return out;
    }

    inline MatrixNxMBase& operator=(const MatrixNxMBase& rhs)
    {
      if (this == &rhs)
        return *this;
      data = rhs.data;
      return *this;
    }

    inline MatrixNxMBase<T, N, M> operator*(T rhs) const
    {
      T d[size];
      for (size_t i = 0; i < size; i++)
        d[i] = data[i]*rhs;
      return MatrixNxMBase<T, N, M>(d);
    }

    inline MatrixNxMBase<T, N, M> operator+(const MatrixNxMBase& rhs) const
    {
      T d[size];
      for (size_t i = 0; i < size; i++)
        d[i] = data[i] + rhs(i);
      return MatrixNxMBase<T, N, M>(d);
    }

    inline MatrixNxMBase<T, N, M> operator-() const
    {
      T d[size];
      for (size_t i = 0; i < size; i++)
        d[i] = -data[i];
      return MatrixNxMBase<T, N, M>(d);
    }

    inline MatrixNxMBase<T, N, M> operator-(const MatrixNxMBase& rhs) const
    {
      T d[size];
      for (size_t i = 0; i < size; i++)
        d[i] = data[i] - rhs(i);
      return MatrixNxMBase<T, N, M>(d);
    }

    inline MatrixNxMBase<T, N, N> operator*(const MatrixNxMBase<T, M, N>& rhs) const
    {
      T d[N*N];
      for (size_t i = 0; i < N; i++)
        for (size_t j = 0; j < N; j++)
          d[N*i + j] = this->row(i)^rhs.col(j);
      return MatrixNxMBase<T, N, N>(d);
    }

    inline VectorNBase<T, N> operator*(const VectorNBase<T, M>& rhs) const
    {
      T d[nrows];
      for (size_t i = 0; i < nrows; i++)
        d[i] = this->row(i)^rhs;
      return VectorNBase<T, N>(d);
    }

    inline MatrixNxMBase<T, N, M> operator%(const MatrixNxMBase& rhs) const
    {
      T d[size];
      for (size_t i = 0; i < size; i++)
        d[i] = data[i]*rhs(i);
      return MatrixNxMBase<T, N, M>(d);
    }

    inline MatrixNxMBase<T, N, M> operator/(const MatrixNxMBase& rhs) const
    {
      T d[size];
      for (size_t i = 0; i < size; i++)
        d[i] = data[i]/rhs(i);
      return MatrixNxMBase<T, N, M>(d);
    }

    inline MatrixNxMBase<T, N, M> operator+=(const MatrixNxMBase& rhs)
    {
      for (size_t i = 0; i < size; i++)
        data[i] += rhs.data[i];
      return *this;
    }

    inline MatrixNxMBase<T, N, M> operator-=(const MatrixNxMBase& rhs)
    {
      for (size_t i = 0; i < size; i++)
        data[i] -= rhs.data[i];
      return *this;
    }

    inline MatrixNxMBase<T, N, M> operator%=(const MatrixNxMBase& rhs)
    {
      for (size_t i = 0; i < size; i++)
        data[i] *= rhs.data[i];
      return *this;
    }

    inline MatrixNxMBase<T, N, M> operator/=(const MatrixNxMBase& rhs)
    {
      for (size_t i = 0; i < size; i++)
        data[i] /= rhs.data[i];
      return *this;
    }

    inline MatrixNxMBase<T, M, N> trans() const
    {
      T d[size];
      for (size_t i = 0; i < nrows; i++)
        for (size_t j = 0; j < ncols; j++)
          d[nrows*j + i] = data[ncols*i + j];
      return MatrixNxMBase<T, M, N>(d);
    }

    inline MatrixNxMBase<T, M, N> t() const
    {
      return this->trans();
    }

    inline VectorNBase<T, M> row(unsigned int r) const
    {
      T d[ncols];
      for (size_t i = 0; i < ncols; i++)
        d[i] = data[ncols*r + i];
      return VectorNBase<T, M>(d);
    }

    inline VectorNBase<T, N> col(unsigned int c) const
    {
      T d[nrows];
      for (size_t i = 0; i < nrows; i++)
        d[i] = data[ncols*i + c];
      return VectorNBase<T, N>(d);
    }

    inline void ones()
    {
      data.fill(1);
    }

    inline void zeros()
    {
      data.fill(0);
    }

    inline MatrixNxMBase<T, N, M> scale(T s) const
    {
      return (*this)*s;
    }

    inline void print(const std::string& prefix = std::string()) const
    {
      if (!prefix.empty())
        std::cout << prefix << std::endl;
      std::cout << (*this) << std::endl;
    }

    inline bool operator==(const MatrixNxMBase& that) const
    {
      return this->equals(that);
    }

    inline bool operator!=(const MatrixNxMBase& that) const
    {
      return !this->equals(that);
    }

    virtual inline bool equals(const MatrixNxMBase& that, const T ptol = 1e-8) const
    {
      return (*this - that).norm() < ptol;
    }

    inline T norm() const
    {
      return math::sqrt((this->trans()*(*this)).trace());
    }

    inline T trace()
    {
      size_t count = nrows <= ncols ? nrows : ncols;
      T tr = 0;
      for (size_t i = 0; i < count; i++)
        tr += data[ncols*i + i];
      return tr;
    }
  };

  template<size_t N, size_t M>
  inline MatrixNxMBase<float, N, M> operator*(const float& lhs,
                                              const MatrixNxMBase<float, N, M>& rhs)
  {
    return rhs*lhs;
  }

  template<size_t N, size_t M>
  inline MatrixNxMBase<double, N, M> operator*(const double& lhs,
                                               const MatrixNxMBase<double, N, M>& rhs)
  {
    return rhs*lhs;
  }

  template<typename T, size_t N, size_t M>
  inline std::ostream& operator<<(std::ostream& out, const MatrixNxMBase<T, N, M>& m)
  {
    for (size_t i = 0; i < N; i++)
    {
      for (size_t j = 0; j < M; j++)
        out << m.data[M*i + j] << " ";
      out << std::endl;
    }
    return out;
  }

  template<typename T, size_t N, size_t M>
  inline MatrixNxMBase<T, M, N> trans(const MatrixNxMBase<T, N, M>& m)
  {
    return m.trans();
  }

  template<typename T, size_t N, size_t M>
  inline MatrixNxMBase<T, N, M> outer(const VectorNBase<T, N>& v1,
                                      const VectorNBase<T, M>& v2)
  {
    T d[N*M];
    for (size_t i = 0; i < N; i++)
      for (size_t j = 0; j < M; j++)
        d[M*i + j] = v1(i)*v2(j);
    return MatrixNxMBase<T, N, M>(d);
  }

  /* *********************************************************************************** */
  template<typename T, size_t N>
  struct MatrixNxNBase : MatrixNxMBase<T, N, N>
  {
    MatrixNxNBase() : MatrixNxMBase<T, N, N>() { }
    MatrixNxNBase(T val) : MatrixNxMBase<T, N, N>(val) { }
    MatrixNxNBase(const MatrixNxNBase& in) : MatrixNxMBase<T, N, N>(in.data) { }
    MatrixNxNBase(const boost::array<T, N*N>& in) : MatrixNxMBase<T, N, N>(in) { }
    MatrixNxNBase(T (&in)[N*N]) : MatrixNxMBase<T, N, N>(in) { }
    MatrixNxNBase(const MatrixNxMBase<T, N, N>& in) : MatrixNxMBase<T, N, N>(in) { }

    inline void eye()
    {
      this->data.fill(0);
      for (size_t i = 0; i < this->nrows; i++)
        this->data[this->nrows*i + i] = 1;
    }

    virtual inline T det() const
    {
      std::cerr << "MatrixNxMBase::det not implemented" << std::endl;
      return T();
    }

    virtual inline MatrixNxNBase<T, N> inv() const
    {
      std::cerr << "MatrixNxMBase::inv not implemented" << std::endl;
      return MatrixNxNBase<T, N>();
    }

    static inline MatrixNxNBase<T, N> diagmat(const VectorNBase<T, N>& in)
    {
      T d[N*N] = { 0 };
      for (size_t i = 0; i < N; i++)
        d[N*i + i] = in(i);
      return MatrixNxNBase<T, N>(d);
    }
  };

  template<size_t N>
  inline MatrixNxNBase<float, N> operator*(const float& lhs,
                                           const MatrixNxNBase<float, N>& rhs)
  {
    return MatrixNxNBase<float, N>(rhs*lhs);
  }

  template<size_t N>
  inline MatrixNxNBase<double, N> operator*(const double& lhs,
                                            const MatrixNxNBase<double, N>& rhs)
  {
    return MatrixNxNBase<double, N>(rhs*lhs);
  }

  template<typename T, size_t N>
  inline MatrixNxNBase<T, N> inv(const MatrixNxNBase<T, N>& m)
  {
    return m.inv();
  }

  /* *********************************************************************************** */
  template<typename T, size_t N>
  struct RotationNBase : MatrixNxNBase<T, N>
  {
    RotationNBase() : MatrixNxNBase<T, N>()
    {
      this->eye();
    }

    RotationNBase(const RotationNBase& in) : MatrixNxNBase<T, N>(in.data) { }
    RotationNBase(const boost::array<T, N*N>& in) : MatrixNxNBase<T, N>(in) { }
    RotationNBase(T (&in)[N*N]) : MatrixNxNBase<T, N>(in) { }
    RotationNBase(const MatrixNxNBase<T, N>& in) : MatrixNxNBase<T, N>(in) { }

    virtual inline MatrixNxNBase<T, N> inv() const
    {
      return this->trans();
    }
  };

  template<size_t N>
  inline RotationNBase<float, N> operator*(const float& lhs,
                                           const RotationNBase<float, N>& rhs)
  {
    return RotationNBase<float, N>(rhs*lhs);
  }

  template<size_t N>
  inline RotationNBase<double, N> operator*(const double& lhs,
                                            const RotationNBase<double, N>& rhs)
  {
    return RotationNBase<double, N>(rhs*lhs);
  }

  template<typename T, size_t N>
  inline RotationNBase<T, N> inv(const RotationNBase<T, N>& m)
  {
    return m.inv();
  }

  /* *********************************************************************************** */
  template<typename T>
  struct Matrix2x2Base : MatrixNxNBase<T, 2>
  {
    Matrix2x2Base() : MatrixNxNBase<T, 2>() { }
    Matrix2x2Base(T val) : MatrixNxNBase<T, 2>(val) { }
    Matrix2x2Base(const Matrix2x2Base& in) : MatrixNxNBase<T, 2>(in.data) { }
    Matrix2x2Base(const boost::array<T, 4>& in) : MatrixNxNBase<T, 2>(in) { }
    Matrix2x2Base(T (&in)[2*2]) : MatrixNxNBase<T, 2>(in) { }
    Matrix2x2Base(const MatrixNxNBase<T, 2>& in) : MatrixNxNBase<T, 2>(in) { }
    Matrix2x2Base(const MatrixNxMBase<T, 2, 2>& in) : MatrixNxNBase<T, 2>(in) { }

    Matrix2x2Base(T R11, T R12, T R21, T R22)
    {
      this->data[0] = R11;
      this->data[1] = R12;
      this->data[2] = R21;
      this->data[3] = R22;
    }

    virtual inline T det() const
    {
      T a = this->data[0];
      T b = this->data[1];
      T c = this->data[2];
      T d = this->data[3];
      return (-(b*c) + a*d);
    }

    virtual inline MatrixNxNBase<T, 2> inv() const
    {
      Vector2Base<T> e(singularValues());

      T emax = e(0);
      T emin = e(1);

      if (emin < std::numeric_limits<T>::denorm_min())
        throw std::runtime_error("Matrix2x2Base: appears singular");

      if (emax/emin > std::numeric_limits<T>::epsilon())
      {
        T a = this->data[0];
        T b = this->data[1];
        T c = this->data[2];
        T d = this->data[3];

        T tmp[4] = {d/(-b*c + a*d), b/(b*c - a*d), c/(b*c - a*d), a/(-b*c + a*d)};
        return MatrixNxNBase<T, 2>(tmp);
      }
      else
        throw std::runtime_error("Matrix2x2Base: appears singular");
    }

    virtual inline Vector2Base<T> singularValues() const
    {
      T a = this->data[0];
      T b = this->data[1];
      T c = this->data[2];
      T d = this->data[3];

      T tmp1 = a*a + b*b + c*c + d*d;
      T tmp2 = math::sqrt((math::pow(b + c, static_cast<T>(2)) +
                           math::pow(a - d, static_cast<T>(2)))*
                          (math::pow(b - c, static_cast<T>(2)) +
                           math::pow(a + d, static_cast<T>(2))));

      T e1 = math::sqrt(tmp1 - tmp2)*M_SQRT1_2;
      T e2 = math::sqrt(tmp1 + tmp2)*M_SQRT1_2;

      return Vector2Base<T>(e1 > e2 ? e1 : e2, e1 < e2 ? e1 : e2);
    }
  };

  inline Matrix2x2Base<float> operator*(const float& lhs, const Matrix2x2Base<float>& rhs)
  {
    return Matrix2x2Base<float>(rhs*lhs);
  }

  inline Matrix2x2Base<double> operator*(const double& lhs, const Matrix2x2Base<double>& rhs)
  {
    return Matrix2x2Base<double>(rhs*lhs);
  }

  typedef Matrix2x2Base<float> Matrix2x2f;
  typedef Matrix2x2Base<float> Mat22f;

  typedef Matrix2x2Base<double> Matrix2x2d;
  typedef Matrix2x2Base<double> Mat22d;

  typedef Matrix2x2Base<double> Matrix2x2;
  typedef Matrix2x2Base<double> Mat22;

  /* *********************************************************************************** */
  template<typename T>
  struct Rotation2Base : RotationNBase<T, 2>
  {
    Rotation2Base() : RotationNBase<T, 2>() { }
    Rotation2Base(const Rotation2Base& in) : RotationNBase<T, 2>(in.data) { }
    Rotation2Base(const boost::array<T, 4>& in) : RotationNBase<T, 2>(in) { }
    Rotation2Base(T (&in)[2*2]) : RotationNBase<T, 2>(in) { }
    Rotation2Base(const RotationNBase<T, 2>& in) : RotationNBase<T, 2>(in) { }
    Rotation2Base(const Matrix2x2Base<T>& in) : RotationNBase<T, 2>(in) { }
    Rotation2Base(const MatrixNxMBase<T, 2, 2>& in) : RotationNBase<T, 2>(in) { }

    Rotation2Base(T val)
    {
      fromAngle(val);
    }

    Rotation2Base(T R11, T R12,
                  T R21, T R22)
    {
      this->data[0] = R11;
      this->data[1] = R12;
      this->data[2] = R21;
      this->data[3] = R22;
    }

    virtual inline bool equals(const Rotation2Base& that, const T ptol = 1e-8) const
    {
      return error(that) < ptol;
    }

    inline T error(const Rotation2Base& r) const
    {
      return math::sin(angle() - r.angle());
    }

    inline T angle() const
    {
      return math::atan2(this->data[2], this->data[0]);
    }

    inline void fromAngle(T val)
    {
      this->data[0] = math::cos(val);
      this->data[1] = -math::sin(val);
      this->data[2] = math::sin(val);
      this->data[3] = math::cos(val);
    }
  };

  inline Rotation2Base<float> operator*(const float& lhs, const Rotation2Base<float>& rhs)
  {
    return Rotation2Base<float>(rhs.scale(lhs));
  }

  inline Rotation2Base<double> operator*(const double& lhs, const Rotation2Base<double>& rhs)
  {
    return Rotation2Base<double>(rhs.scale(lhs));
  }

  typedef Rotation2Base<float> Rotation2f;
  typedef Rotation2Base<float> Rot2f;

  typedef Rotation2Base<double> Rotation2d;
  typedef Rotation2Base<double> Rot2d;

  typedef Rotation2Base<double> Rotation2;
  typedef Rotation2Base<double> Rot2;

  /* *********************************************************************************** */
  template<typename T>
  struct Transform2Base
  {
    typedef boost::shared_ptr<Transform2Base> Ptr;
    typedef boost::shared_ptr<const Transform2Base> ConstPtr;

    Vector2Base<T> translation;
    Rotation2Base<T> rotation;

    Transform2Base()
    {
      translation.zeros();
      rotation.eye();
    }

    Transform2Base(const Vector2Base<T>& translation_,
                   const Rotation2Base<T>& rotation_) :
      translation(translation_), rotation(rotation_) { }

    Transform2Base(const Transform2Base<T>& in) :
      translation(in.translation), rotation(in.rotation) { }

    Transform2Base(T x, T y, T th) : translation(x, y), rotation(th) {}

    Transform2Base& operator=(const Transform2Base& rhs)
    {
      if (this == &rhs)
        return *this;
      translation = rhs.translation;
      rotation = rhs.rotation;
      return *this;
    }

    Vector2Base<T> operator*(const Vector2Base<T>& p) const
    {
      return rotation*p + translation;
    }

    Transform2Base<T> operator+(const Transform2Base<T>& t) const
    {
      return Transform2Base<T>(translation + rotation*t.translation,
                               rotation*t.rotation);
    }

    bool operator==(const Transform2Base& that) const
    {
      return this->equals(that);
    }

    bool operator!=(const Transform2Base& that) const
    {
      return !this->equals(that);
    }

    bool equals(const Transform2Base& that,
                const T ptol = 1e-5,
                const T rtol = 1e-5) const
    {
      return (translation.equals(that.translation, ptol) &&
              rotation.equals(that.rotation, rtol));
    }

    void print(const std::string& prefix = std::string()) const
    {
      if (!prefix.empty())
        std::cout << prefix << std::endl;
      std::cout << (*this) << std::endl;
    }

    static Transform2Base identity()
    {
      return Transform2Base();
    }
  };

  template<typename T>
  std::ostream& operator<<(std::ostream& out, const Transform2Base<T>& m)
  {
    out << "translation:" << std::endl << m.translation << std::endl;
    out << "rotation:" << std::endl << m.rotation;
    return out;
  }

  template<typename T>
  Transform2Base<T> pose_update(const Transform2Base<T>& t1,
                                const Transform2Base<T>& t2)
  {
    return Transform2Base<T>(t1.translation + t1.rotation*t2.translation,
                             t1.rotation*t2.rotation);
  }

  template<typename T>
  Transform2Base<T> pose_inverse(const Transform2Base<T>& t)
  {
    return Transform2Base<T>(-1.0*t.rotation.trans()*t.translation,
                             t.rotation.trans());
  }

  template<typename T>
  Transform2Base<T> pose_delta(const Transform2Base<T>& t1,
                               const Transform2Base<T>& t2)
  {
    return Transform2Base<T>(t1.rotation.trans()*(t2.translation - t1.translation),
                             t1.rotation.trans()*t2.rotation);
  }

  typedef Transform2Base<float> Transform2f;
  typedef Transform2Base<double> Transform2d;
  typedef Transform2d Transform2;
  typedef Transform2 Tr2;

  /* *********************************************************************************** */
  inline double unroll(double x)
  {
    x = fmod(x, 2.0*M_PI);
    if (x < 0) x += 2.0*M_PI;
    return x;
  }

  /* *********************************************************************************** */
  inline double normalize(double x)
  {
    x = fmod(x + M_PI, 2.0*M_PI);
    if (x < 0) x += 2.0*M_PI;
    return x - M_PI;
  }

  /* *********************************************************************************** */
  inline double shortest_angular_distance(double from, double to)
  {
    double result = unroll(unroll(to) - unroll(from));
    if (result > M_PI)
      result = -(2.0*M_PI - result);
    return normalize(result);
  }

  /* *********************************************************************************** */
  inline double rad2deg(double angle)
  {
    return angle*180.0*M_1_PI;
  }

  /* *********************************************************************************** */
  inline double deg2rad(double angle)
  {
    return angle*M_PI/180.0;
  }
};

#endif

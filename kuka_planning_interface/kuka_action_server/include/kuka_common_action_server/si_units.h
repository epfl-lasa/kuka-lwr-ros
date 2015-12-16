#ifndef SI_UNITS_H_
#define SI_UNITS_H_
/*
#include <iostream>


template<int M, int K, int S> struct Unit{
    enum{m=M,kg=K,s=S};
};

template <typename Unit>
struct Value{
    double val;
    explicit constexpr Value():val(0){}
    explicit constexpr Value(double d):val(d){}

    Value& operator+=(Value& rhs)
    {
        val+=rhs.val;
        return *this;
    }
};

using speed         = Value< Unit<1,0,-1> >;
using acceleration  = Value< Unit<1,0,-2> >;
using second        = Unit<0,0,1>;
using second2       = Unit<0,0,2>;
using meters        = Unit<1,0,0>;

constexpr Value<meters> operator"" _m(long double d){
    return Value<meters>(static_cast<double>(d));
}

constexpr Value<::second> operator"" _s(long double d){
    return Value<second>(static_cast<double>(d));
}

constexpr Value<second2> operator"" _s2(long double d){
    return Value<second2>(static_cast<double>(d));
}

*/

#endif

#include <boost/bind.hpp>

#define MEM_FN(x) boost::bind(&self_type::x, shared_from_this())

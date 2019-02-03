#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <g2o/types/sba/types_direct_method.h>

#include <g2o/types/slam3d/se3quat.h>
//#include "python/core/base_vertex.h"
#include "python/core/base_unary_edge.h"
#include "python/core/base_binary_edge.h"
#include "python/core/base_multi_edge.h"


namespace py = pybind11;
using namespace pybind11::literals;


namespace g2o {

void declareTypesDirectMethod(py::module & m) {

      // project a 3d point into an image plane, the error is photometric error
      // an unary edge with one vertex SE3Expmap (the pose of camera)
      templatedBaseUnaryEdge< 1, double, VertexSE3Expmap>(m, "BaseUnaryEdge_1_double_VertexSE3Expmap");
	  py::class_<EdgeSE3ProjectDirect, BaseUnaryEdge< 1, double, VertexSE3Expmap>>(m, "EdgeSE3ProjectDirect")
		  .def(py::init<Eigen::Vector3d, float, float, float, float, pybind11::array_t<unsigned char> >())
		  .def("compute_error", &EdgeSE3ProjectDirect::computeError)
		  .def("linearize_oplus", &EdgeSE3ProjectDirect::linearizeOplus)
	  ;

}

}  // end namespace g2o

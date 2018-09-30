/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "open_karto/Mapper.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

karto::Dataset* CreateMap(karto::Mapper* pMapper)
{
  karto::Dataset* pDataset = new karto::Dataset();

  /////////////////////////////////////
  // Create a laser range finder device - use SmartPointer to let karto subsystem manage memory.
  karto::Name name("laser0");
  karto::LaserRangeFinder* pLaserRangeFinder = karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, name);
  pLaserRangeFinder->SetOffsetPose(karto::Pose2(1.0, 0.0, 0.0));
  pLaserRangeFinder->SetAngularResolution(karto::math::DegreesToRadians(0.5));
  pLaserRangeFinder->SetRangeThreshold(12.0);

  pDataset->Add(pLaserRangeFinder);

  /////////////////////////////////////
  // Create three localized range scans, all using the same range readings, but with different poses. 
  karto::LocalizedRangeScan* pLocalizedRangeScan = NULL;

  // 1. localized range scan

  // Create a vector of range readings. Simple example where all the measurements are the same value.
  std::vector<kt_double> readings;
  for (int i=0; i<361; i++)
  {
    readings.push_back(3.0);
  }

  // create localized range scan
  pLocalizedRangeScan = new karto::LocalizedRangeScan(name, readings);
  pLocalizedRangeScan->SetOdometricPose(karto::Pose2(0.0, 0.0, 0.0));
  pLocalizedRangeScan->SetCorrectedPose(karto::Pose2(0.0, 0.0, 0.0));

  // Add the localized range scan to the mapper
  pMapper->Process(pLocalizedRangeScan);
  std::cout << "Pose: " << pLocalizedRangeScan->GetOdometricPose() << " Corrected Pose: " << pLocalizedRangeScan->GetCorrectedPose() << std::endl;

  // Add the localized range scan to the dataset
  pDataset->Add(pLocalizedRangeScan);

  // 2. localized range scan

  // create localized range scan
  pLocalizedRangeScan = new karto::LocalizedRangeScan(name, readings);
  pLocalizedRangeScan->SetOdometricPose(karto::Pose2(1.0, 0.0, 1.57));
  pLocalizedRangeScan->SetCorrectedPose(karto::Pose2(1.0, 0.0, 1.57));

  // Add the localized range scan to the mapper
  pMapper->Process(pLocalizedRangeScan);
  std::cout << "Pose: " << pLocalizedRangeScan->GetOdometricPose() << " Corrected Pose: " << pLocalizedRangeScan->GetCorrectedPose() << std::endl;

  // Add the localized range scan to the dataset
  pDataset->Add(pLocalizedRangeScan);

  // 3. localized range scan

  // create localized range scan
  pLocalizedRangeScan = new karto::LocalizedRangeScan(name, readings);
  pLocalizedRangeScan->SetOdometricPose(karto::Pose2(1.0, -1.0, 2.35619449));
  pLocalizedRangeScan->SetCorrectedPose(karto::Pose2(1.0, -1.0, 2.35619449));

  // Add the localized range scan to the mapper
  pMapper->Process(pLocalizedRangeScan);
  std::cout << "Pose: " << pLocalizedRangeScan->GetOdometricPose() << " Corrected Pose: " << pLocalizedRangeScan->GetCorrectedPose() << std::endl;

  // Add the localized range scan to the dataset
  pDataset->Add(pLocalizedRangeScan);

  return pDataset;
}

karto::OccupancyGrid* CreateOccupancyGrid(karto::Mapper* pMapper, kt_double resolution)
{
  std::cout << "Generating map..." << std::endl;

  // Create a map (occupancy grid) - time it
  karto::OccupancyGrid* pOccupancyGrid = karto::OccupancyGrid::CreateFromScans(pMapper->GetAllProcessedScans(), resolution);

  return pOccupancyGrid;
}

void PrintOccupancyGrid(karto::OccupancyGrid* pOccupancyGrid)
{
  if (pOccupancyGrid != NULL)
    {
    // Output ASCII representation of map
    kt_int32s width = pOccupancyGrid->GetWidth();
    kt_int32s height = pOccupancyGrid->GetHeight();
    karto::Vector2<kt_double> offset = pOccupancyGrid->GetCoordinateConverter()->GetOffset();

    std::cout << "width = " << width << ", height = " << height << ", scale = " << pOccupancyGrid->GetCoordinateConverter()->GetScale() << ", offset: " << offset.GetX() << ", " << offset.GetY() << std::endl;
    for (kt_int32s y=height-1; y>=0; y--)
    {
      for (kt_int32s x=0; x<width; x++) 
      {
        // Getting the value at position x,y
        kt_int8u value = pOccupancyGrid->GetValue(karto::Vector2<kt_int32s>(x, y));

        switch (value)
        {
        case karto::GridStates_Unknown:
          std::cout << "*";
          break;
        case karto::GridStates_Occupied:
          std::cout << "X";
          break;
        case karto::GridStates_Free:
          std::cout << " ";
          break;
        default:
          std::cout << "?";
        }
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
}

void test() {
  karto::Mapper* pMapper = new karto::Mapper();
  if (pMapper != NULL)
    {
      karto::OccupancyGrid* pOccupancyGrid = NULL;
      std::cout << "Tutorial 1 ----------------" << std::endl << std::endl;
      pMapper->Reset();
      karto::Dataset* pDataset = CreateMap(pMapper);
      pOccupancyGrid = CreateOccupancyGrid(pMapper, 0.1);
      PrintOccupancyGrid(pOccupancyGrid);
      delete pOccupancyGrid;
      delete pMapper;
      delete pDataset;
    }
}

namespace py = pybind11;

// karto::LaserRangeFinder* CreateCustomRangeFinder(karto::Name name) {
//   return karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, name);
// }


kt_bool ProcessLocalizedRangeScan(karto::Mapper * mapper, karto::LocalizedRangeScan * scan) {
  return mapper->Process(scan);
}

void AddRangeFinderToDataset(karto::LaserRangeFinder * rf, karto::Dataset * ds) {
  ds->Add(rf);
}

void AddLocalizedRangeScanToDataset(karto::LocalizedRangeScan * rf, karto::Dataset * ds) {
  ds->Add(rf);
}


PYBIND11_MODULE(openkarto, m) {
  // m.def("create_custom_rangefinder", &CreateCustomRangeFinder);
  m.def("process_localized_range_scan", &ProcessLocalizedRangeScan);
  m.def("add_range_finder_to_dataset", &AddRangeFinderToDataset);
  m.def("add_localized_range_scan_to_dataset", &AddLocalizedRangeScanToDataset);
  m.def("create_occupancy_grid", &CreateOccupancyGrid);

  py::class_<karto::Mapper>(m, "Mapper")
    .def(py::init<>())
    .def("reset", &karto::Mapper::Reset)
    .def_property("min_travel_distance", &karto::Mapper::getParamMinimumTravelDistance, &karto::Mapper::setParamMinimumTravelDistance)
    .def_property("min_travel_heading", &karto::Mapper::getParamMinimumTravelHeading, &karto::Mapper::setParamMinimumTravelHeading)
  .def("get_processed_scans", &karto::Mapper::GetAllProcessedScans);
    // .def("process_scan", py::overload_cast<karto::LocalizedRangeScan *>(&karto::Mapper::Process), py::const_);

  py::class_<karto::Dataset>(m, "Dataset")
    .def(py::init<>())
    .def("add", &karto::Dataset::Add);

  py::class_<karto::Pose2>(m, "Pose2")
    .def(py::init<double, double, double>())
    .def_property("x", &karto::Pose2::GetX, &karto::Pose2::SetX)
    .def_property("y", &karto::Pose2::GetY, &karto::Pose2::SetY)
    .def_property("yaw", &karto::Pose2::GetHeading, &karto::Pose2::SetHeading)
    .def("__repr__", [](const karto::Pose2 &a) {
        std::stringstream buffer;
        buffer << "(x: " << a.GetX() << ", y: " << a.GetY() << ", heading: " << a.GetHeading() << ")\n";
        return buffer.str();
      })
    ;

  py::class_<karto::Vector2<kt_double> >(m, "Vec2_d")
    .def(py::init<double, double>())
    .def_property("x", &karto::Vector2<kt_double>::GetX, &karto::Vector2<kt_double>::SetX)
    .def_property("y", &karto::Vector2<kt_double>::GetY, &karto::Vector2<kt_double>::SetY)
    .def("__repr__", [](const karto::Vector2<kt_double> &a) {
        std::stringstream buffer;
        buffer << "(x: " << a.GetX() << ", y:" << a.GetY() << ")\n";
        return buffer.str();
      });

  py::class_<karto::Name>(m, "Name")
    .def(py::init<const std::string &>());

  py::class_<karto::LaserRangeFinder>(m, "LaserRangeFinder")
    .def(py::init(&karto::LaserRangeFinder::CreateLaserRangeFinder))
    .def("set_offset_pose", &karto::LaserRangeFinder::SetOffsetPose)
    .def("set_angular_resolution", &karto::LaserRangeFinder::SetAngularResolution)
    .def("set_minimum_range", &karto::LaserRangeFinder::SetMinimumRange)
    .def("set_minimum_angle", &karto::LaserRangeFinder::SetMinimumAngle)
    .def("set_maximum_range", &karto::LaserRangeFinder::SetMaximumRange)
    .def("set_maximum_angle", &karto::LaserRangeFinder::SetMaximumAngle)
    .def("set_angular_resolution", &karto::LaserRangeFinder::SetAngularResolution)
    .def("set_range_threshold", &karto::LaserRangeFinder::SetRangeThreshold);
  // TODO min/max angle???

  py::class_<karto::LocalizedRangeScan>(m, "LocalizedRangeScan")
    .def(py::init<karto::Name, std::vector<kt_double> >())
    .def("set_odometric_pose", &karto::LocalizedRangeScan::SetOdometricPose)
    .def("get_odometric_pose", &karto::LocalizedRangeScan::GetOdometricPose)
    .def("set_corrected_pose", &karto::LocalizedRangeScan::SetCorrectedPose)
    .def("get_corrected_pose", &karto::LocalizedRangeScan::GetCorrectedPose);

  py::enum_<karto::LaserRangeFinderType>(m, "LaserRangeFinderType")
    .value("Custom", karto::LaserRangeFinderType::LaserRangeFinder_Custom);

  py::enum_<karto::GridStates>(m, "GridStates")
    .value("Unknown", karto::GridStates::GridStates_Unknown)
    .value("Occupied", karto::GridStates::GridStates_Occupied)
    .value("Free", karto::GridStates::GridStates_Free);

  py::class_<karto::OccupancyGrid>(m, "OccupancyGrid")
    .def_property_readonly("width", &karto::OccupancyGrid::GetWidth)
    .def_property_readonly("height", &karto::OccupancyGrid::GetHeight)
    .def_property_readonly("offset", [](const karto::OccupancyGrid &a){
        auto offset = (a.GetCoordinateConverter()->GetOffset());
        return offset;
      })
    .def("get_value", [](const karto::OccupancyGrid &a, kt_int32s x, kt_int32s y) {
        return a.GetValue(karto::Vector2<kt_int32s>(x, y));
      })
    ;

#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif
}

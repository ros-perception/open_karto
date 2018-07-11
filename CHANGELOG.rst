^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package open_karto
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2018-07-11)
------------------
* Adds maintainer and branches for melodic
* Merge pull request `#14 <https://github.com/ros-perception/open_karto/issues/14>`_ from ros-perception/maintainer-add
  Adding myself as a maintainer for open_karto
* Adding myself as a maintainer for open_karto
* Merge pull request `#11 <https://github.com/ros-perception/open_karto/issues/11>`_ from Maidbot/minimum_time_interval
  Process scan if enough time has elapsed since the previous one
* Merge pull request `#10 <https://github.com/ros-perception/open_karto/issues/10>`_ from mikepurvis/fix-cpp11
  Use std::isnan/isinf, for C++11 compatibility.
* [Mapper] Take time into account in HasMoveEnough
  The function HasMovedEnough now also returns true if more than MinimumTimeInterval time has elapsed since the previously processed laser scan
* [Mapper] Add new parameter, MinimumTimeInterval
* Use std::isnan/isinf, for C++11 compatibility.
* Merge pull request `#8 <https://github.com/ros-perception/open_karto/issues/8>`_ from mig-em/feature/invalidScans1.1.4
  Add invalid scan detection for scanmatcher
* Merge pull request `#9 <https://github.com/ros-perception/open_karto/issues/9>`_ from Maidbot/ignore_min_readings
  Ignore readings less than the sensor's minimum range
* [Karto] Also ignore readings less than the minimum range
* Add invalid scan detection for scanmatcher
* Contributors: Luc Bettaieb, Michael Ferguson, Mike Purvis, Russell Toris, Spyros Maniatopoulos, mig-em

1.1.4 (2016-03-02)
------------------
* update build status badges
* Adds LocalizedRangeScanWithPoints range scan
* Contributors: Michael Ferguson, Russell Toris

1.1.3 (2016-02-16)
------------------
* Link against, and export depend on, boost
* Contributors: Hai Nguyen, Michael Ferguson

1.1.2 (2015-07-13)
------------------
* Added getters and setters for parameters inside Mapper so they can be changed via the ROS param server.
* Contributors: Luc Bettaieb, Michael Ferguson

1.1.1 (2015-05-07)
------------------
* Makes FindValidPoints robust to the first point in the scan being a NaN
* Bump minimum cmake version requirement
* Fix cppcheck warnings
* Fix newlines (dos2unix) & superfluous whitespace
* Protect functions that throw away const-ness (check dirty flag) with mutex
* Don't modify scan during loop closure check - work on a copy of it
* removed useless return to avoid cppcheck error
* Add Mapper::SetUseScanMatching
* Remove html entities from log output
* Fix NANs cause raytracing to take forever
* Contributors: Daniel Pinyol, Michael Ferguson, Paul Mathieu, Siegfried-A. Gevatter Pujals, liz-murphy

1.1.0 (2014-06-15)
------------------
* Release as a pure catkin ROS package

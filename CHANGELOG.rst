^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package open_karto
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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

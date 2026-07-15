^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kuka_rsi_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2026-07-15)
------------------
* Synchronize update rate of controller manager 
* Add option to lock memory
* Change model verification to only consider payload, reach and type 
* Scheduling diagnostics 
* Add external axis support 
* Make CPU affinities and realtime thread priority configurable
* Add mxAutomation support 
* [Fix] Deactivate plain RSI driver in case of an error

1.0.0 (2025-10-03)
------------------
* Redesign RSI communication to make it real-time safe
* Add EKI wrapper for starting RSI
* Add IO support
* Add iiQKA.OS2 support
* Add Gazebo support
* Smaller bugfixes
* Contributors: Komáromi Sándor, Kristof Pasztor, Áron Svastits, Levente Nas, Kristófi Mihály

0.9.2 (2024-07-10)
------------------
* Fix GCC warning causing unstable build

0.9.1 (2024-07-08)
------------------
* Add missing test dependency

0.9.0 (2024-07-08)
------------------
* Add package with driver for KUKA KSS robots
* Contributors: Aron Svastits, Gergely Kovacs, Marton Antal, Lars Tingelstad

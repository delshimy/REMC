#!/usr/bin/sclang

/***********************************************************************

    Reactive Environment for Network Music Performance
    Copyright (C) <2013>  Dalia El-Shimy

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

***********************************************************************/

g.waitForBoot {
(
q.current_directory = File.getcwd;
catArgs(q.current_directory,"/src/initial_parameters.scd").load;
catArgs(q.current_directory,"/src/gui_controls.scd").load;
catArgs(q.current_directory,"/src/gui_main.scd").load;
catArgs(q.current_directory,"/src/gui_update.scd").load;
catArgs(q.current_directory,"/src/kinect_calibration.scd").load;
catArgs(q.current_directory,"/src/start_performance.scd").load;
catArgs(q.current_directory,"/src/stop_performance.scd").load;

q.createGUI_main;
);
}.play;




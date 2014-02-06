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


q.initial_parameters = { |q|

	q.x_ratio = 1; 
	q.y_ratio = 1;
	q.x_total = 1600;//Window.availableBounds.right;
	q.y_total = 900;//Window.availableBounds.bottom;
	q.dv=false;
	q.cond_dv = "dv_off";
	q.tp=false;
	q.cond_tp = "tp_off";
	q.ms=false;
	q.cond_ms = "ms_off";
	q.dr=false;
	q.cond_dr = "dr_false";
	q.mc=false;
	q.cond_mc = "mc_false";
	q.save=false; //no need to save steve's data
	q.num=3;
	q.cond_string="";

	q.y_available = q.y_total-72;
	q.x_available = q.x_total-48;
	q.y_ratio = 1;
	q.x_ratio = 1;
	q.y_total;
	q.min_effective_ratio=[q.x_ratio,q.y_ratio].minItem;
	

	thisProcess.openUDPPort(57120);
	thisProcess.openUDPPort(57121);

	//for starters we get our working directory
	q.current_directory = File.getcwd;

	q.player_num = 0;

	q.base_volume_local=Array.newClear(q.num);
	q.base_reverb_local=Array.newClear(q.num);

	q.volume_image_value=Array.newClear(q.num);
	q.reverb_image_value=Array.newClear(q.num);


	q.connected=false;

	
};	
	

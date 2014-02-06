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


q.stopjam = { |q|

	q.num.do({arg i;
		q.volume_slider[i].enabled = false;
		q.reverb_slider[i].enabled = false;
	});
	q.calibrate_button.enabled=true;
	q.calibrate_label.stringColor=Color.black;	
	
	q.volume_range_slider.value=0;
	q.volume_range_slider.enabled = false;

	q.reverb_range_slider.value=0;
	q.reverb_range_slider.enabled = false;

	q.started = false;

	if(q.save,{
		q.file_position.close;
		q.file_orientation.close;
		q.file_settings.close;
	});



	q.player_out_right.free;
	q.player_out_left.free;
	Server.all.do(_.freeAll);
	
	"jmess -D".unixCmd;
	"killall osceleton_for_NMP".unixCmd;

	//now we remove all osc nodes
	q.info_osc.remove;
	b.close;

};

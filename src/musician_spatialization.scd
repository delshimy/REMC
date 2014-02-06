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


q.musician_spatialization = { |q, ms_angle, volume, player_positions_viewer|

	var right_to_left = Array.newClear(2);
	var volume_right = Array.newClear(q.num);
	var volume_left = Array.newClear(q.num);

	right_to_left[0]=(-1)*cos(ms_angle);
	right_to_left[1]=sin(ms_angle);
	q.num.do({arg i;
		var unit_vector=[0,0];
		var dot_product;
		if(i==q.player_num,{
			volume_right[i]=volume[i];
			volume_left[i]=volume[i];
		},{
			unit_vector[0]=(player_positions_viewer[q.player_num,0]-player_positions_viewer[i,0])/(player_positions_viewer.rowAt(q.player_num)-player_positions_viewer.rowAt(i)).squared.sum.sqrt;
			unit_vector[1]=(player_positions_viewer[q.player_num,1]-player_positions_viewer[i,1])/(player_positions_viewer.rowAt(q.player_num)-player_positions_viewer.rowAt(i)).squared.sum.sqrt;
			dot_product = (unit_vector[0]*right_to_left[0])+(unit_vector[1]*right_to_left[1]);
			volume_right[i]=(volume[i]/2)*((1+dot_product).pow(2));
			volume_left[i]=(volume[i]/2)*((1-dot_product).pow(2));
			
		});


	});

	[volume_right, volume_left];
};


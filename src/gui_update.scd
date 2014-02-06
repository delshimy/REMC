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




q.updateGUI = { |q, player_positions_viewer, distance, distance_to_monitors, head_roll, body_yaw|


	var color_array;
	var vertex_0 = Array2D.new(q.num,2);
	var vertex_1 = Array2D.new(q.num,2);
	var vertex_2 = Array2D.new(q.num,2);
	var vertex_3 = Array2D.new(q.num,2);
	var lshoulder_centre = Array2D.new(q.num,2);
	var rshoulder_centre = Array2D.new(q.num,2);
	var arc_num_volume=Array.newClear(q.num);
	var arc_num_reverb=Array.newClear(q.num);
	var arc_max=10;
	var arc_mid=3;
	var angles = Array.newClear(q.num);
	var head_position =Array2D.new(q.num,2);
	var body_yaw_reflected=Array.newClear(q.num);
	var body_yaw_adjusted=Array.newClear(q.num);
	var body_yaw_rotated=Array.newClear(q.num);
	var head_roll_adjusted=Array.newClear(q.num);
	var head_offset, label_offset_x, label_offset_y;
	var label_positions=Array2D.new(q.num,2);
	
	
	//this is how much we need to move the labels by to make them appear in the viewer
	label_offset_x=q.x_available-q.viewer_total_width-25-(q.head_radius/2);
	label_offset_y=q.head_radius/2;	

	color_array= [[0.8,0,0],[0,0.8,0],[0,0,0.8]];

	q.num.do({arg i;
		body_yaw_reflected[i]=0;
		body_yaw_rotated[i]=0;
		arc_num_volume[i]=0;
		arc_num_reverb[i]=0;
	});

	q.num.do({arg i;
		

		body_yaw_adjusted[i]=body_yaw[i];
		head_roll_adjusted[i]=head_roll[i]*(-1);

		head_offset=head_roll_adjusted[i].linlin(-pi/4,pi/4, (-1)*q.shoulder_length, q.shoulder_length, \minmax).floor;


		if(q.mc,{	
			head_position[i,0]=player_positions_viewer[i,0]+head_offset;
		},{
			head_position[i,0]=player_positions_viewer[i,0];
		});

		head_position[i,1]=player_positions_viewer[i,1];	
		

		if(i==q.player_num,{
				arc_num_reverb[i] = 0;
				arc_num_volume[i] = 0;
		},{

			if(q.dv,{
				var temp1, temp2;				

				temp1=distance_to_monitors[i,q.player_num].linlin(0,q.max_dist/2, arc_mid.ampdb, 1.ampdb,\minmax).dbamp.floor;

				if(distance_to_monitors[q.player_num,i]<(q.volume_threshold_near*q.max_dist/2),{
					temp2=arc_max-arc_mid;
				});


				if((distance_to_monitors[q.player_num,i]>(q.volume_threshold_near*q.max_dist/2))&&(distance_to_monitors[q.player_num,i]<(q.volume_threshold_far*q.max_dist/2)),{
					temp2=distance[i].linlin(distance_to_monitors[i,q.player_num]+(q.volume_threshold_near*q.max_dist/2), distance_to_monitors[i,q.player_num]+(q.volume_threshold_far*q.max_dist/2),7.ampdb, 1.ampdb).dbamp.floor;
				});

				if(distance_to_monitors[q.player_num,i]>(q.volume_threshold_far*q.max_dist/2),{
					temp2=0;
				});
				arc_num_volume[i] = temp1+temp2;
			});			


			if(q.dr,{
				arc_num_reverb[i] = distance[i].linlin(((q.reverb_safe_threshold+1)*q.max_dist/2),q.max_dist, 1, arc_max, \minmax).floor;
			});
		});
		

		vertex_0[i,0]=player_positions_viewer[i,0]-q.shoulder_length;
		vertex_0[i,1]=player_positions_viewer[i,1]-q.shoulder_width;

		vertex_1[i,0]=player_positions_viewer[i,0]+q.shoulder_length;
		vertex_1[i,1]=player_positions_viewer[i,1]-q.shoulder_width;

		vertex_2[i,0]=player_positions_viewer[i,0]-q.shoulder_length;
		vertex_2[i,1]=player_positions_viewer[i,1]+q.shoulder_width;	

		vertex_3[i,0]=player_positions_viewer[i,0]+q.shoulder_length;	
		vertex_3[i,1]=player_positions_viewer[i,1]+q.shoulder_width;

		lshoulder_centre[i,0]=player_positions_viewer[i,0]-q.shoulder_length;
		lshoulder_centre[i,1]=player_positions_viewer[i,1];

		rshoulder_centre[i,0]=player_positions_viewer[i,0]+q.shoulder_length;
		rshoulder_centre[i,1]=player_positions_viewer[i,1];

		if(i==q.player_num,{
			body_yaw_reflected[i]=(body_yaw_adjusted[i]+pi);
		},{
			body_yaw_reflected[i]=body_yaw_adjusted[i];
		});
		
		q.num.do({arg i;
			body_yaw_rotated[i]=body_yaw_reflected[i]+q.body_yaw_rot_angles[i];
		});

		c.drawFunc = { 	
			//Pen.translate(shoulder_width,shoulder_width);
			Pen.width=5;		
			q.num.do({arg i;
				Pen.rotate(body_yaw_rotated[i], player_positions_viewer[i,0], player_positions_viewer[i,1]);
			

				
				Pen.line(vertex_2[i,0]@vertex_2[i,1], vertex_3[i,0]@vertex_3[i,1]);
				Pen.addArc(lshoulder_centre[i,0]@lshoulder_centre[i,1], q.shoulder_width, 0.5*pi , pi);
				Pen.addArc(rshoulder_centre[i,0]@rshoulder_centre[i,1], q.shoulder_width, 1.5*pi, pi);
				Pen.fillColor=Color.fromArray(color_array[i]);
				Pen.fill;


				Pen.addArc(head_position[i,0]@head_position[i,1], q.head_radius, 0.0, 2pi);
				Pen.fillColor=Color.new255(255,228,196);
				Pen.fill;

				Pen.width=3;
				

				//Pen.color=Color.white;
				Pen.color=Color.black;
				arc_num_reverb[i].do({arg j;
					Pen.addArc(head_position[i,0]@(head_position[i,1]+(2*q.head_radius)+(10*(arc_num_reverb[i]+1))), q.head_radius+(10*(j+1)), -pi/3, (-pi/3) );
					Pen.perform(\stroke);
				});

				//Pen.color=Color.black;
				arc_num_volume[i].do({arg j;
					if((j<(arc_max/3).floor)&&(j>=0),{
						Pen.color=Color.green(1.0,1.0);
					});
					if((j<(2*arc_max/3).floor)&&(j>=(arc_max/3).floor),{
						Pen.color=Color.yellow(1.0, 1.0);
					});
					if(j>=(2*arc_max/3).floor,{
						Pen.color=Color.red(1.0, 1.0);
					});

					Pen.addArc(head_position[i,0]@head_position[i,1], q.head_radius+(10*(j+1)),pi/3, pi/3 );
					Pen.perform(\stroke);
				});

				Pen.rotate((-1)*body_yaw_rotated[i], player_positions_viewer[i,0], player_positions_viewer[i,1]);	
				
					
			});

		};
		if(q.mc,{
			label_positions[i,0]=player_positions_viewer[i,0]+(head_offset*cos(body_yaw_rotated[i]))+label_offset_x;
			label_positions[i,1]=player_positions_viewer[i,1]+(head_offset*sin(body_yaw_rotated[i]))+label_offset_y;
		},{
			label_positions[i,0]=player_positions_viewer[i,0]+label_offset_x;
			label_positions[i,1]=player_positions_viewer[i,1]+label_offset_y;
		});		
		q.player_labels[i].bounds=Rect(label_positions[i,0], label_positions[i,1],q.head_radius,q.head_radius);		
	});

};




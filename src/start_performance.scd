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

************************************************************************/

q.startjam = { |q|
	
	var file_names=Array.newClear(3);
	var out_channels=[0,1]; //note that structure is [left, right]

	//we define all variables related to volume changes
	var base_volume = Array.newClear(q.num);
	var min_volume = Array.newClear(q.num); 
	var volume = Array.newClear(q.num);
	var max_volume_slider=75;


	//we define all variables related to reverb
	var min_reverb = 0.6;
	var max_reverb_slider = 7;
	var max_reverb_range = 7;
	var max_reverb_total = max_reverb_slider+max_reverb_range;
	var base_reverb = Array.newClear(q.num);
	var max_reverb = Array.newClear(q.num);
	var reverb = Array.newClear(q.num);
	
	var min_damping = 0.1;
	var max_damping = 0.9;
	var damping = Array.newClear(q.num);


	//we define all variables related to distances
	var player_positions=Array2D.new(q.num,2);
	var player_positions_adjusted_kinect_location=Array2D.new(q.num,2);
	var player_positions_translated=Array2D.new(q.num,2);
	var player_positions_rotated=Array2D.new(q.num,2);
	var player_positions_translated_for_GUI=Array2D.new(q.num,2);
	var player_positions_rotated_for_GUI=Array2D.new(q.num,2);
	var player_positions_rotated_perspective=Array2D.new(q.num,2);
	var player_positions_translated_viewer=Array2D.new(q.num,2);
	var player_positions_scaled=Array2D.new(q.num,2);
	var player_positions_viewer=Array2D.new(q.num,2);
	var player_received, info_received;
	var distance = Array.newClear(q.num);
	//these are to be used if video is on
	var distance_to_monitors=Array2D.new(q.num, q.num);
	var monitor_position = Array2D.new(q.num,q.num);

	//we define all variables related to ti, mc and ms
	var tp_angle = 0;
	var ms_angle = 0;
	var mc_angle=0;
	var head_roll = Array.newClear(q.num);
	var head_yaw = Array.newClear(q.num);
	var body_yaw = Array.newClear(q.num);
	var volume_left = Array.newClear(q.num);
	var volume_right = Array.newClear(q.num);
	

	var filter_size=20;
	var filter_head_roll=Array2D.new(q.num,filter_size);
	var filter_head_yaw=Array2D.new(q.num,filter_size);
	var filter_body_yaw=Array2D.new(q.num,filter_size);
	var filter_x_pos = Array2D.new(q.num,filter_size);
	var filter_y_pos = Array2D.new(q.num,filter_size);
	var temp_x, temp_y;
	var counter=0;	
	var kinect_command_without_window;


	//start the kinect
	kinect_command_without_window=catArgs("cd OSCeleton_for_NMP &&./osceleton_for_NMP -xt -p ", NetAddr.langPort);
	kinect_command_without_window.unixCmd;	

	q.createGUI_controls();

	q.volume_threshold_near = 0.5;
	q.volume_threshold_far = 0.666;
	q.reverb_safe_threshold = 0.3;

	q.started = true;
	q.num.do({arg i;
		q.volume_slider[i].enabled=true;
	});

	q.num.do({arg i;
		q.reverb_slider[i].enabled=true;
	});



	if(q.dv,{
		q.volume_range_slider.enabled=true;
		q.volume_range_slider.value=0.5;
	});
	if(q.dr,{
		q.reverb_range_slider.enabled=true;
		q.reverb_range_slider.value=0.5;
	});
	
	q.calibrate_button.enabled=false;

	q.current_time=0;	
	q.tp_active=false;

	if(q.save,{
		//we get the current time
		q.start_time=thisThread.clock.beats*1000;

		//we set up our files for saving
		file_names[0]=catArgs(q.current_directory,"/data/position/Musician_",q.player_num,"_", q.cond_dv,"_",q.cond_dr, "_", q.cond_ti,"_", q.cond_mc, "_", q.cond_ms, "_position_", Date.getDate, ".txt");
		file_names[1]=catArgs(q.current_directory,"/data/orientation/Musician_",q.player_num,"_", q.cond_dv,"_",q.cond_dr, "_", q.cond_ti,"_", q.cond_mc, "_", q.cond_ms, "_orientation_", Date.getDate, ".txt");
		file_names[2]=catArgs(q.current_directory,"/data/settings/Musician_",q.player_num,"_", q.cond_dv,"_",q.cond_dr, "_", q.cond_ti,"_", q.cond_mc, "_", q.cond_ms, "_settings_", Date.getDate, ".txt");	

		q.file_position=File(file_names[0].asString,"w");
		q.file_orientation=File(file_names[1].asString,"w");
		q.file_settings=File(file_names[2].asString,"w");
	});

	q.num.do({arg i;
		q.num.do({arg j;
			monitor_position[i,j]=[0,0];
		});
	});		

	//for player 0, left is player 1, right is player 2
	monitor_position[0,1]=q.kinect_corners_rotated[0,2]; //left monitor
	monitor_position[0,2]=q.kinect_corners_rotated[0,3]; //right monitor
	//for player 1, left is player 2, right is player 0
	monitor_position[1,2]=q.kinect_corners_rotated[1,2]; //left monitor
	monitor_position[1,0]=q.kinect_corners_rotated[1,3]; //right monitor
	//for player 2, left is player 0, right is player 1
	monitor_position[2,0]=q.kinect_corners_rotated[2,2]; //left monitor
	monitor_position[2,1]=q.kinect_corners_rotated[2,3]; //right monitor

	
	
	q.num.do({arg i;
		if(q.base_volume_local[i]==nil,{
			q.base_volume_local[i]=0.00001;
		});

		if(q.base_reverb_local[i]==nil,{
			q.base_reverb_local[i]=min_reverb;
		});

		if(q.volume_image_value[i]==nil,{
			q.volume_image_value[i]=0;
		});
	
		if(q.reverb_image_value[i]==nil,{
			q.reverb_image_value[i]=0;
		});
	});



	if(q.volume_range==nil,{
		q.volume_range = 0.5;
	});
	if(q.reverb_range==nil,{
		q.reverb_range = max_reverb_range/2; //we want to make the total range between 0 and 3. This way, the maximum reverb experienced is at 5.
	});

	q.num.do({arg i;
		q.volume_icon[i].drawFunc = { 
        	   	Pen.image(q.volume_image[i][q.volume_image_value[i], q.reverb_image_value[i]]); 
   		};
	});	
		
	//max_reverb=min_reverb+q.reverb_range;



	//initialize our arrays
	q.num.do({arg i;
		base_volume[i]=q.base_volume_local[i];
		volume[i]=base_volume[i];
		min_volume[i]=volume[i]*q.volume_range;


		base_reverb[i]=q.base_reverb_local[i];
		reverb[i]=base_reverb[i];
		max_reverb[i]=reverb[i]+q.reverb_range;
		
		damping[i]=(((reverb[i]-min_reverb)/(max_reverb_total-min_reverb))*(min_damping-max_damping))+max_damping;


		head_roll[i]=0;
		head_yaw[i]=0;
		body_yaw[i]=0;

		
		//we want all avatars off the viewer until they start sending messages
		player_positions[i,0]=q.kinect_corners_actual[i,1][0];
		player_positions[i,1]=q.kinect_corners_actual[i,1][1];
		player_positions_adjusted_kinect_location[i,0]=0;
		player_positions_adjusted_kinect_location[i,1]=0;
		player_positions_translated[i,0]=0;
		player_positions_translated[i,1]=0;
		player_positions_translated_for_GUI[i,0]=0;
		player_positions_translated_for_GUI[i,1]=0;
		player_positions_rotated[i,0]=q.kinect_corners_rotated[i,1][0];
		player_positions_rotated[i,1]=q.kinect_corners_rotated[i,1][1];
		player_positions_rotated_for_GUI[i,0]=q.kinect_corners_rotated[i,1][0];
		player_positions_rotated_for_GUI[i,1]=q.kinect_corners_rotated[i,1][1];
		//initialize player_positions_viewer in case we are not receiving messages from other players yet
		player_positions_rotated_perspective[i,0]=(q.rotation_matrix[q.player_num][0,0]*player_positions_rotated_for_GUI[i,0])+(q.rotation_matrix[q.player_num][0,1]*player_positions_rotated_for_GUI[i,1]);
		player_positions_rotated_perspective[i,1]=(q.rotation_matrix[q.player_num][1,0]*player_positions_rotated_for_GUI[i,0])+(q.rotation_matrix[q.player_num][1,1]*player_positions_rotated_for_GUI[i,1]); 
		player_positions_translated_viewer[i,0]=player_positions_rotated_perspective[i,0]-q.min_x;
		player_positions_translated_viewer[i,1]=player_positions_rotated_perspective[i,1]-q.min_y;
		
		player_positions_scaled[i,0] = player_positions_translated_viewer[i,0]*q.pos_viewer_ratio_x;
		player_positions_scaled[i,1] = player_positions_translated_viewer[i,1]*q.pos_viewer_ratio_y; 
		player_positions_viewer[i,0] = player_positions_scaled[i,0]+q.avatar_margin;
		player_positions_viewer[i,1] = q.viewer_total_height-player_positions_scaled[i,1]-q.avatar_margin;

		volume_right[i]=0;
		volume_left[i]=0;
		q.num.do({arg j;
			distance_to_monitors[i,j]=0;
		});
	});




	q.num.do({arg i;
		filter_size.do({arg j;
			filter_head_roll[i,j]=0;
			filter_head_yaw[i,j]=0;
			filter_body_yaw[i,j]=0;
			filter_x_pos[i,j]=q.kinect_corners_actual[i,1][0];
			filter_y_pos[i,j]=q.kinect_corners_actual[i,1][1];
		});
	});


	/*q.num.do({arg i;
		base_volume[i]=q.base_volume_local[i];			
		volume[i]=q.base_volume_local[i];
		//volume_left[i]=q.base_volume_local[i];
		//volume_right[i]=q.base_volume_local[i];
	});*/	

	t=Task.new({
		catArgs(q.current_directory,"/src/synth_definition.scd").load;		
		l.sync;
	
		//we initialize our local player synth
		q.player_out_left = Synth(\player_mix, [\amp_instrument, volume, \channel_out, out_channels[0], \roomsize, 16, \revtime_instrument, reverb, \damp_instrument, damping]);
		q.player_out_right = Synth(\player_mix, [\amp_instrument, volume, \channel_out, out_channels[1], \roomsize, 16, \revtime_instrument, reverb, \damp_instrument, damping]);	
		"we've initialized our synth now".postln;		
	});

	t.play;	


	
	
	
	//now we get the volume for all three musicians	
	q.num.do({arg i;
		q.volume_slider[i].action_({|v,x,y,m| 
			base_volume[i]=(v.value*max_volume_slider+0.00001);
			q.base_volume_local[i]=base_volume[i];
			base_volume.postln;
			volume[i]=base_volume[i];
			min_volume[i] = base_volume[i]*q.volume_range;
			q.player_out_right.set(\amp_instrument, base_volume);
			q.player_out_left.set(\amp_instrument, base_volume);
			if(q.save,{
				q.current_time=thisThread.clock.beats*1000-q.start_time;		
				q.file_settings.putString(scatArgs("volume", q.current_time.asString, base_volume[q.player_num].asString,"\n"));
			});

	
			if(q.volume_slider[i].value==0,{
        	   			q.volume_image_value[i]=0;
			});
			if((q.volume_slider[i].value<=0.33)&&(q.volume_slider[i].value>0),{
        	   			q.volume_image_value[i]=1; 
			}); 	
			if((q.volume_slider[i].value>0.33)&&(q.volume_slider[i].value<=0.66),{
					q.volume_image_value[i]=2;
			});
			if(q.volume_slider[i].value>0.66,{
					q.volume_image_value[i]=3;
			});

			q.volume_icon[i].drawFunc = { 
           			Pen.image(q.volume_image[i][q.volume_image_value[i], q.reverb_image_value[i]]); 
   			};		
	
		}).value=(q.base_volume_local[i]/max_volume_slider);
	});


	q.num.do({arg i;
		q.reverb_slider[i].action_({|v,x,y,m| 
			base_reverb[i]=(v.value*(max_reverb_slider-min_reverb))+min_reverb;
			q.base_reverb_local[i]=base_reverb[i];
			base_reverb.postln;
			reverb[i]=base_reverb[i];
			max_reverb[i] = base_reverb[i] + q.reverb_range;

			damping[i]=(((reverb[i]-min_reverb)/(max_reverb_total-min_reverb))*(min_damping-max_damping))+max_damping;
			damping.postln;
			q.player_out_right.set(\revtime_instrument, reverb, \damp_instrument, damping);
			q.player_out_left.set(\revtime_instrument, reverb, \damp_instrument, damping);
			if(q.save,{
				q.current_time=thisThread.clock.beats*1000-q.start_time;		
				q.file_settings.putString(scatArgs("reverb", q.current_time.asString, base_reverb[q.player_num].asString,"\n"));
			});

	
			if(q.reverb_slider[i].value==0,{
        	   			q.reverb_image_value[i]=0;
			});
			if((q.reverb_slider[i].value<=0.33)&&(q.reverb_slider[i].value>0),{
        	   			q.reverb_image_value[i]=1; 
			}); 	
			if((q.reverb_slider[i].value>0.33)&&(q.reverb_slider[i].value<=0.66),{
					q.reverb_image_value[i]=2;
			});
			if(q.reverb_slider[i].value>0.66,{
					q.reverb_image_value[i]=3;
			});

			q.volume_icon[i].drawFunc = { 
           			Pen.image(q.volume_image[i][q.volume_image_value[i], q.reverb_image_value[i]]); 
   			};		
	
		}).value=((q.base_reverb_local[i]-min_reverb)/max_reverb_slider);
	});



			
	//now we define actions for the slider
	q.volume_range_slider.action_({|v,x,y,m| 
		q.volume_range=(1-v.value+0.0001);
		
		if(q.save,{
			q.current_time=thisThread.clock.beats*1000-q.start_time;		
			q.file_settings.putString(scatArgs("volume_range", q.current_time.asString, q.volume_range.asString,"\n"));
		});

		q.num.do({arg i;
			min_volume[i]=base_volume[i]*q.volume_range;
		});
		min_volume.postln;
					
	}).value=(1-q.volume_range+0.0001);




	q.reverb_range_slider.action_({|v,x,y,m| 
		q.reverb_range=v.value*max_reverb_range;
		
		if(q.save,{
			q.current_time=thisThread.clock.beats*1000-q.start_time;		
			q.file_settings.putString(scatArgs("reverb_range", q.current_time.asString, q.reverb_range.asString,"\n"));
		});

		q.num.do({arg i;
			max_reverb[i]=base_reverb[i]+q.reverb_range;
		});
		
	}).value=(q.reverb_range/max_reverb_range);

	//we want to define our ms and ti values;
	catArgs(q.current_directory,"/src/track_panning.scd").load;
	catArgs(q.current_directory,"/src/musician_spatialization.scd").load;
	catArgs(q.current_directory,"/src/mix_control.scd").load;
	
	//In a co-ordinate system where z points "in/out", y points "up/down", x points "left/right"
	//We are interested in x and z values.
	//The message format is as follows:
	//[ /client (0), body_part(1), player_id (2), X(3), Z(4), Y(5), pitch(6), yaw(7), roll(8)]
	//Note that we receive the messages for all three players
	q.info_osc = OSCresponderNode(nil, '/client', {arg time, resp, msg;
			
		//var player_received;
		//msg.postln;
		
		if (msg[2]==(-1),{
			var limited_x, limited_y, scaled_x, scaled_y;
			//this means it is coming from the local computer	
			player_received=q.player_num;
			//we limit the range base_volumed on our calibration results
			if(msg[3]<=q.kinect_min_x_actual,{
				limited_x=q.kinect_min_x_actual;
			},{
				if(msg[3]>=q.kinect_max_x_actual,{
					limited_x=q.kinect_max_x_actual;
				},{
					limited_x=msg[3];
				});
			});

			if(msg[5]<=q.kinect_min_y_actual,{
				limited_y=q.kinect_min_y_actual;
			},{
				if(msg[5]>=q.kinect_max_y_actual,{
					limited_y=q.kinect_max_y_actual;
				},{
					limited_y=msg[5];	
				});
			});

			//we scale it from the kinect's actual parameters to the kinect's ideal parameters before sending it back out
			scaled_x = (((limited_x-q.kinect_min_x_actual)*(q.kinect_max_x_ideal-q.kinect_min_x_ideal))/(q.kinect_max_x_actual-q.kinect_min_x_actual))+q.kinect_min_x_ideal;
			scaled_y = (((limited_y-q.kinect_min_y_actual)*(q.kinect_max_y_ideal-q.kinect_min_y_ideal))/(q.kinect_max_y_actual-q.kinect_min_y_actual))+q.kinect_min_y_ideal;

			msg[3] = scaled_x;
			msg[5] = scaled_y;					

		},{
			player_received=msg[2];

		});		
		
	
		if(msg[1].asString=="head",{
			filter_head_yaw[player_received, counter%filter_size]=msg[7]*pi/180;
			filter_head_roll[player_received, counter%filter_size]=msg[8]*pi/180;		
		});

		if(msg[1].asString=="torso",{
			filter_x_pos[player_received, counter%filter_size]=msg[3];
			filter_y_pos[player_received, counter%filter_size]=msg[5];

			filter_body_yaw[player_received, counter%filter_size]=msg[7]*pi/180;
					
			head_roll[player_received]=((filter_head_roll.rowAt(player_received).sum)/filter_size);
			head_yaw[player_received]=((filter_head_yaw.rowAt(player_received).sum)/filter_size);
			body_yaw[player_received]=((filter_body_yaw.rowAt(player_received).sum)/filter_size);
			
			
	
			//ms_angle=((-1)*(head_yaw[player_received]-body_yaw[player_received]));
			if(q.ms,{			
				ms_angle=(-1)*(body_yaw[player_received]);
			},{
				ms_angle=0;
			});
			if(q.tp,{
				tp_angle=(-1)*(body_yaw[player_received]);
			},{
				tp_angle=0;
			});
			if(q.mc,{
				mc_angle=(-1)*(body_yaw[player_received]);
			},{

				mc_angle=0;
			});
	

			player_positions[player_received,0]=((filter_x_pos.rowAt(player_received).sum)/filter_size);
			player_positions[player_received,1]=((filter_y_pos.rowAt(player_received).sum)/filter_size);

			counter=counter+1;
			
			player_positions_adjusted_kinect_location[player_received,0]=player_positions[player_received,0];
			player_positions_adjusted_kinect_location[player_received,1]=(q.kinect_min_y_ideal+q.kinect_max_y_ideal)-player_positions[player_received,1];

			
			player_positions_translated[player_received,0]=player_positions[player_received,0]+q.translation[player_received,0];
			player_positions_translated[player_received,1]=player_positions[player_received,1]+q.translation[player_received,1];

			player_positions_translated_for_GUI[player_received,0]=player_positions_adjusted_kinect_location[player_received,0]+q.translation[player_received,0];
			player_positions_translated_for_GUI[player_received,1]=player_positions_adjusted_kinect_location[player_received,1]+q.translation[player_received,1];


			
			player_positions_rotated[player_received,0]=(q.rotation_matrix[player_received][0,0]*player_positions_translated[player_received,0])+(q.rotation_matrix[player_received][0,1]*player_positions_translated[player_received,1]);
			player_positions_rotated[player_received,1]=(q.rotation_matrix[player_received][1,0]*player_positions_translated[player_received,0])+(q.rotation_matrix[player_received][1,1]*player_positions_translated[player_received,1]);

			
			player_positions_rotated_for_GUI[player_received,0]=(q.rotation_matrix[player_received][0,0]*player_positions_translated_for_GUI[player_received,0])+(q.rotation_matrix[player_received][0,1]*player_positions_translated_for_GUI[player_received,1]);
			player_positions_rotated_for_GUI[player_received,1]=(q.rotation_matrix[player_received][1,0]*player_positions_translated_for_GUI[player_received,0])+(q.rotation_matrix[player_received][1,1]*player_positions_translated_for_GUI[player_received,1]);



			//we need to make the adjustments base_volumed on the current player's perspective
			player_positions_rotated_perspective[player_received,0]=(q.rotation_matrix[q.player_num][0,0]*player_positions_rotated_for_GUI[player_received,0])+(q.rotation_matrix[q.player_num][0,1]*player_positions_rotated_for_GUI[player_received,1]);
			player_positions_rotated_perspective[player_received,1]=(q.rotation_matrix[q.player_num][1,0]*player_positions_rotated_for_GUI[player_received,0])+(q.rotation_matrix[q.player_num][1,1]*player_positions_rotated_for_GUI[player_received,1]); 

			

			//positions have already been rotated and are in the appropriate quadrant
			//we still need to translate, then scale them so that they can be seen by our viewer
			player_positions_translated_viewer[player_received,0]=player_positions_rotated_perspective[player_received,0]-q.min_x;
			player_positions_translated_viewer[player_received,1]=player_positions_rotated_perspective[player_received,1]-q.min_y;
		
			player_positions_scaled[player_received,0] = player_positions_translated_viewer[player_received,0]*q.pos_viewer_ratio_x;
			player_positions_scaled[player_received,1] = player_positions_translated_viewer[player_received,1]*q.pos_viewer_ratio_y; 
			player_positions_viewer[player_received,0] = player_positions_scaled[player_received,0]+q.avatar_margin;
			player_positions_viewer[player_received,1] = q.viewer_total_height-player_positions_scaled[player_received,1]-q.avatar_margin;


			//now we need to calculate the distances to each monitor
			if((q.dv||q.dr),{					
				q.num.do({arg i;
					q.num.do({arg j;
						if(i!=j,{
							distance_to_monitors[i,j]=(player_positions_rotated_perspective.rowAt(i)-monitor_position[i,j]).squared.sum.sqrt;
						});
					});
				});

				q.num.do({arg i;
					distance[i]=distance_to_monitors[i,q.player_num]+distance_to_monitors[q.player_num,i];	
				});
			});
		
					
			q.num.do({arg i;
				if(i!=q.player_num,{
					if(q.dv,{
						var vol_other;
						vol_other=distance_to_monitors[i,q.player_num].linlin(0,q.max_dist/2, (0.5*base_volume[i]).ampdb, 1.ampdb).dbamp;
						if(distance_to_monitors[q.player_num,i]<(q.volume_threshold_near*q.max_dist/2),{
								volume[i]=base_volume[i]+vol_other;
						});
						if((distance_to_monitors[q.player_num,i]>(q.volume_threshold_near*q.max_dist/2))&&(distance_to_monitors[q.player_num,i]<(q.volume_threshold_far*q.max_dist/2)),{
							volume[i]=distance[i].linlin(distance_to_monitors[i,q.player_num]+(q.volume_threshold_near*q.max_dist/2), distance_to_monitors[i,q.player_num]+(q.volume_threshold_far*q.max_dist/2),(base_volume[i]).ampdb, min_volume[i].ampdb, \minmax).dbamp+vol_other;	
						});
						if(distance_to_monitors[q.player_num,i]>(q.volume_threshold_far*q.max_dist/2),{
							volume[i]=min_volume[i]+vol_other;	
						});
					},{
						volume[i]=base_volume[i];		
					});

					if(q.dr,{
						if(distance_to_monitors[q.player_num,i]>(q.reverb_safe_threshold*(q.max_dist/2)),{
							reverb[i]=distance[i].linlin(((q.reverb_safe_threshold+1)*q.max_dist/2),q.max_dist,base_reverb[i], max_reverb[i]);
						},{
							reverb[i]=base_reverb[i];
						});							
					},{
						reverb[i]=base_reverb[i];
					});
				
					damping[i]=(((reverb[i]-min_reverb)/(max_reverb_total-min_reverb))*(min_damping-max_damping))+max_damping;
				});		
				
			});
			("volume"+volume).postln;
			("reverb"+reverb).postln;
			
			//[volume[1],reverb[1]].postln;
			
			q.num.do({arg i;
				volume_left[i]=volume[i];		
				volume_right[i]=volume[i];
			});

			if(player_received==q.player_num,{
				if(q.save,{
					q.current_time=thisThread.clock.beats*1000-q.start_time;		
					q.file_position.putString(scatArgs(q.current_time.asString, player_positions[q.player_num,0].asString, player_positions			[q.player_num,1].asString, player_positions_rotated[q.player_num,0].asString, player_positions_rotated[q.player_num,1].asString, "\n"));
					q.file_orientation.putString(scatArgs(q.current_time.asString, head_roll[q.player_num].asString, head_yaw[q.player_num].asString, body_yaw[q.player_num].asString, "\n"));
				});

				if(q.ms,{
					var temp = Array.newClear(2);
					temp = q.musician_spatialization(ms_angle, volume, player_positions_viewer);
					volume_right = temp[0];
					volume_left = temp[1];	
				});

			
				if(q.tp,{
					var temp = Array.newClear(2);
					temp = q.track_panning(tp_angle, player_positions_viewer, volume_right, volume_left);
					volume_right = temp[0];
					volume_left = temp[1];
				});

				if(q.mc,{
					var temp = Array.newClear(2);
					temp = q.mix_control(mc_angle, player_positions_viewer, volume_right, volume_left);
					volume_right = temp[0];
					volume_left = temp[1];
				});

			
			});	

			//[volume_left[2], volume_right[2]].postln;
			q.player_out_right.set(\amp_instrument, volume_right, \revtime_instrument, reverb, \damp_instrument, damping);
			q.player_out_left.set(\amp_instrument, volume_left, \revtime_instrument, reverb, \damp_instrument, damping);

			q.updateGUI(player_positions_viewer, distance, distance_to_monitors, head_roll, body_yaw);
	
		});
								
	}).add;
				
};


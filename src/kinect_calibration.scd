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

q.kinect_parameters_part_1 = {|q|
	
	q.kinect_min_x_ideal = 0.2;
	q.kinect_max_x_ideal = 0.8;
	q.kinect_min_y_ideal = 0.7;
	q.kinect_max_y_ideal = 1.3;

	q.kinect_params_file_name = catArgs(q.current_directory,"/settings/kinect_parameters_local.txt");


	if(File.exists(q.kinect_params_file_name),{
		//this means a calibration has been performed in the past
		var temp_string = "";
		var temp_params_values = Array.newClear(4);
		var temp_params_string;
		var counter = 0;

		q.calibration_performed = true;
		q.kinect_params_file= File(q.kinect_params_file_name,"r");

		temp_params_string = q.kinect_params_file.readAllString;
		temp_params_string.size.do({arg i;
			if(temp_params_string[i].asString!=";",{
				temp_string=catArgs(temp_string.asString,temp_params_string[i].asString);
			},{
				temp_params_values[counter]=temp_string;
				temp_string="";
				counter = counter+1;
			});
		});

		q.kinect_min_x_actual = temp_params_values[0].asFloat;		
		q.kinect_min_y_actual = temp_params_values[1].asFloat;
		q.kinect_max_x_actual = temp_params_values[2].asFloat;
		q.kinect_max_y_actual = temp_params_values[3].asFloat;

		q.kinect_params_file.close;
					
	},{
		//if a file doesn't already exist, it means no calibration has ever been performed.
		//we assign default values
		q.calibration_performed = false;
		q.kinect_min_x_actual = q.kinect_min_x_ideal;
		q.kinect_min_y_actual = q.kinect_min_y_ideal;
		q.kinect_max_x_actual = q.kinect_max_x_ideal;
		q.kinect_max_y_actual = q.kinect_max_y_ideal;
	});

	//we create a second function that is called whenever we calibrate or receive new parameters
	q.kinect_parameters_part_2();

};

q.kinect_parameters_part_2 = {|q|

	var kinect_corners_translated = Array2D.new(q.num,4);
	
	var all_x=Array.newClear(q.num*4);
	var all_y=Array.newClear(q.num*4);

	var rotated_max_height = Array.newClear(q.num); 
	var rotated_max_width = Array.newClear(q.num);

	var effective_viewer_height, effective_viewer_width;
	var max_height, max_width;
	var kinect_width = Array.newClear(q.num);
	var kinect_height = Array.newClear(q.num);
	var counter=0;

	
	q.kinect_corners_rotated=Array2D.new(q.num,4);
	q.kinect_corners_actual = Array2D.new(q.num,4);
	
	q.num.do({arg i;
		4.do({arg j;
			q.kinect_corners_actual[i,j]=[0,0];
			kinect_corners_translated[i,j]=[0,0];
		});
	});

	q.num.do({arg i;
		q.kinect_corners_actual[i,0][0]=q.kinect_max_x_ideal; //0.8
		q.kinect_corners_actual[i,1][0]=q.kinect_min_x_ideal; //0.2
		q.kinect_corners_actual[i,2][0]=q.kinect_min_x_ideal; //0.2
		q.kinect_corners_actual[i,3][0]=q.kinect_max_x_ideal; //0.8

		q.kinect_corners_actual[i,0][1]=q.kinect_min_y_ideal; //0.7
		q.kinect_corners_actual[i,1][1]=q.kinect_min_y_ideal; //0.7
		q.kinect_corners_actual[i,2][1]=q.kinect_max_y_ideal; //1.3
		q.kinect_corners_actual[i,3][1]=q.kinect_max_y_ideal; //1.3
	});	

	q.num.do({arg i;		
		kinect_width[i]=q.kinect_max_x_ideal-q.kinect_min_x_ideal;
		kinect_height[i]=q.kinect_max_y_ideal-q.kinect_min_y_ideal;
	});
	
	q.translation = Array2D.new(q.num,2);

	q.num.do({arg i;
		q.translation[i,0] = (-1)*(q.kinect_min_x_ideal+(kinect_width[i]/2));
		q.translation[i,1] = (-1)*(q.kinect_max_y_ideal+((kinect_width[i]/2)*tan(pi/6)));
	});

	q.num.do({arg i;
		4.do({arg j;
			kinect_corners_translated[i,j][0]=q.kinect_corners_actual[i,j][0]+q.translation[i,0];
			kinect_corners_translated[i,j][1]=q.kinect_corners_actual[i,j][1]+q.translation[i,1];
		});
	});


	q.rotation_matrix = Array.newClear(q.num);
	q.num.do({arg i;
		q.rotation_matrix[i]=Array2D.new(2,2);
	});

	
	q.perspective_angle=Array.newClear(q.num);
		
	q.num.do({arg i;
		q.perspective_angle[i]=((q.num-q.player_num+i)%q.num)*(-120)*pi/180;
	});

	q.perspective_angle.postln;

	q.num.do({arg i;
		q.rotation_matrix[i][0,0]=cos(q.perspective_angle[i]);
		q.rotation_matrix[i][0,1]=(-1)*sin(q.perspective_angle[i]);
		q.rotation_matrix[i][1,0]=sin(q.perspective_angle[i]);
		q.rotation_matrix[i][1,1]=cos(q.perspective_angle[i]);
	});

	q.body_yaw_rot_angles = Array.newClear(q.num);

	q.num.do({arg i;
		if(i!=q.player_num,{
			var temp;
			temp=(q.num-q.player_num+i)%q.num;	
			q.body_yaw_rot_angles[i]=((-1).pow(temp))*(60*pi/180);
		},{
		
			q.body_yaw_rot_angles[i]=0;
		});
	});



	q.num.do({arg i;
		4.do({arg j;
			q.kinect_corners_rotated[i,j]=[0,0];
		});
	});
	
	q.num.do({arg i;
		4.do({arg j;
			q.kinect_corners_rotated[i,j][0]=(q.rotation_matrix[i][0,0]*kinect_corners_translated[i,j][0])+(q.rotation_matrix[i][0,1]*kinect_corners_translated[i,j][1]);
			q.kinect_corners_rotated[i,j][1]=(q.rotation_matrix[i][1,0]*kinect_corners_translated[i,j][0])+(q.rotation_matrix[i][1,1]*kinect_corners_translated[i,j][1]);	
		});
	});

	//now we find the room's mininum and maximum dimensions
	q.num.do({arg i;
		4.do({arg j;
			all_x[(i*4)+j]=q.kinect_corners_rotated[i,j][0];
			all_y[(i*4)+j]=q.kinect_corners_rotated[i,j][1];
		});	
	});	
	
	q.max_y;
	q.kinect_corners_rotated;
	q.min_x=all_x.minItem;
	q.min_y=all_y.minItem;
	q.max_x=all_x.maxItem;	
	q.max_y=all_y.maxItem;


	q.head_radius = 30*q.min_effective_ratio;
	q.shoulder_length = 60*q.min_effective_ratio; //note this is only half the length
	q.shoulder_width = 20*q.min_effective_ratio; //note this is only half the width;

	//this is the margin we should be leaving along in the viewer
	q.avatar_margin=q.shoulder_length+q.shoulder_width;
	effective_viewer_width=q.viewer_total_width-(2*q.avatar_margin);
	effective_viewer_height=q.viewer_total_height-(2*q.avatar_margin);

	max_width = (q.max_x-q.min_x).abs;
	max_height = (q.max_y-q.min_y).abs;


	//now we calculate our new ratios
	q.pos_viewer_ratio_x = effective_viewer_width/max_width;
	q.pos_viewer_ratio_y = effective_viewer_height/max_height;

	q.min_dist = 0;
	q.max_dist = 2*(((q.kinect_max_x_ideal-q.kinect_min_x_ideal).squared+(q.kinect_max_y_ideal-q.kinect_min_y_ideal).squared).sqrt);

	


};


q.kinect_calibration_part_1 = { |q|
	//this function is activated when we press the "calibrate" button. It only pertains to the local user. Values can then be sent to remote users.	
	
	var kinect_command_with_window;
	var counter = 0;
	var counter_limit = 1000;
	var calibration_status;
	var x_array = Array.newClear(counter_limit);
	var y_array = Array.newClear(counter_limit);

	q.calibration_performed = true;

	//first we disconnect to avoid any confusion; 
	q.disconnect();	
	
	//now we disable to the start/connect button
	q.connect_button.enabled=false;
	q.connect_label.stringColor=Color.grey(0.8);
	q.calibrate_button.enabled=false;
	

	//start the kinect
	kinect_command_with_window=catArgs("cd OSCeleton_for_NMP &&./osceleton_for_NMP -xt -p ", NetAddr.langPort," -w");	
	kinect_command_with_window.unixCmd;

	//set up a node to listen to incoming messaged from the kinect.		
	q.kinect_calibration_data= OSCresponder(nil, '/client',{arg time, resp, msg;	
	//The message format is as follows:
	//[ /client (0), body_part(1), player_id (2), X(3), Z(4), Y(5), pitch(6), yaw(7), roll(8)]
		if(msg[2]==(-1),{
			if(counter<counter_limit,{
				x_array[counter]=msg[3];
				y_array[counter]=msg[5];
				counter=counter+1;
				calibration_status = ((counter/counter_limit)*100).asInteger;
				q.calibrate_button.states=[[catArgs("Calibration: ",calibration_status.asString,"\%"),Color.black,Color.new255(255,255,150)],["Start Calibration", Color.black, Color.new255(255,255,150)]];
				q.calibrate_button.refresh;
				counter.postln;
			},{
				"killall osceleton_for_NMP".unixCmd;
				q.kinect_calibration_data.remove; //this might not work?
				q.kinect_calibration_part_2(x_array,y_array);
			});
		});		
		
			
	}).add;

};

q.kinect_calibration_part_2 = { |q, x_array, y_array|
	
	//in this function we write the results of our calibration to file and send it to the other players.	
	

	q.kinect_min_x_actual = x_array.minItem;
	q.kinect_max_x_actual = x_array.maxItem;
	q.kinect_min_y_actual = y_array.minItem;
	q.kinect_max_y_actual = y_array.maxItem;


	//we also want to write this new calibration data to our local file
	q.kinect_params_file = File(q.kinect_params_file_name,"w");
	q.kinect_params_file.write(catArgs(q.kinect_min_x_actual.asString,";",q.kinect_min_y_actual.asString,";",q.kinect_max_x_actual.asString,";",q.kinect_max_y_ideal.asString,";"));
	q.kinect_params_file.close;


	q.connect_button.enabled=true;
	q.connect_label.stringColor=Color.black;
	q.calibrate_button.enabled=true;
	q.calibrate_label.stringColor=Color.black;
	q.calibrate_button.valueAction = 1;
	q.calibrate_label.string = "Calibration complete!";	

	q.kinect_parameters_part_2();

};



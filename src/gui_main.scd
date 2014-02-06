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


q.createGUI_main = { |q|
	var pos_window;
	var connect_label;
	var on_off=0;
	var colors;
	var color_array;
	var volume_range_label, volume_min_label, volume_max_label;
	var reverb_range_label, reverb_min_label, reverb_max_label;
	var left_margin=0;
	var ideal_height; 
	var x_pos,y_pos,x_size,y_size,font_size;
	var volume_slider_label=Array.newClear(q.num);
	var reverb_slider_label=Array.newClear(q.num);

	q.initial_parameters();	

	left_margin = (q.x_available/3).asInteger;
	
	
	w= Window("Reactive Environment for Network Music Performance",Rect(0,0,q.x_available,q.y_total-48)).front;
	w.view.background_(Color.white);	

	
	x_pos =  (125*q.x_ratio).asInteger;
	y_pos = (25*q.y_ratio).asInteger;
	x_size = (300*q.x_ratio).asInteger;
	y_size = (100*q.y_ratio).asInteger;
	font_size = (25*q.min_effective_ratio).asInteger;
	q.calibrate_button = GUI.button.new(w, Rect(x_pos, y_pos, x_size, y_size));
	q.calibrate_button.font=Font("Calibri",font_size, bold:true);
	q.calibrate_button.states=[["Calibration: 0%",Color.black,Color.new255(255,255,150)],["Start Calibration", Color.black, Color.new255(255,255,150)]];
	q.calibrate_button.valueAction=1;	
	
	x_pos = 0;
	y_pos = (125*q.y_ratio).asInteger;
	x_size = left_margin;
	y_size = (50*q.y_ratio).asInteger;
	font_size = (15*q.min_effective_ratio).asInteger;
	q.calibrate_label = StaticText(w, Rect(x_pos, y_pos, x_size, y_size));
	q.calibrate_label.align=\center;
	q.calibrate_label.font = Font("Calibri", font_size, bold:true);		
	q.calibrate_label.string = "Click to calibrate Kinect.";
	
	

	x_pos = (125*q.x_ratio).asInteger;
	y_pos = (200*q.y_ratio).asInteger;
	x_size = (300*q.x_ratio).asInteger;
	y_size = (100*q.y_ratio).asInteger;
	font_size = (25*q.min_effective_ratio).asInteger;
	q.connect_button = GUI.button.new(w, Rect(x_pos, y_pos, x_size, y_size));
	q.connect_button.font=Font("Calibri",font_size, bold:true);
	q.connect_button.states=[[ "Start Performance", Color.black, Color.new255(48,120,0)],[ "Stop Performance", Color.black, Color.new255(162,0,0)]];
	


	q.volume_slider = Array.newClear(q.num);
	q.num.do({arg i;
		var temp = (left_margin-(370*q.x_ratio))/2;
		//x_pos = (((i+1)*(left_margin/4))-((20*q.x_ratio)/2)).asInteger;
		y_pos = (340*q.y_ratio).asInteger;
		x_size = (20*q.x_ratio).asInteger;
		x_pos = (100*q.x_ratio)+(i*temp)+(i*50*q.x_ratio);
		y_size = (250*q.min_effective_ratio).asInteger;
		q.volume_slider[i]=Slider(w,Rect(x_pos, y_pos, x_size, y_size));
		q.volume_slider[i].enabled=false;
	});

	q.num.do({arg i;
		var temp = (left_margin-(370*q.x_ratio))/2;
		y_pos = (605*q.y_ratio).asInteger;
		x_size = (30*q.x_ratio).asInteger;
		y_size = (15*q.min_effective_ratio).asInteger;
		x_pos = (100*q.x_ratio)+(i*temp)+(i*50*q.x_ratio)-(x_size/4);
		font_size = (15*q.min_effective_ratio).asInteger;
		volume_slider_label[i] = StaticText(w, Rect(x_pos, y_pos, x_size, y_size));
		volume_slider_label[i].align=\center;
		volume_slider_label[i].font = Font("Calibri", font_size, bold:true);		
		volume_slider_label[i].string = "Vol";
	});


	q.reverb_slider = Array.newClear(q.num);
	q.num.do({arg i;
		var temp = (left_margin-(370*q.x_ratio))/2;
		//x_pos = (((i+1)*(left_margin/4))-((20*q.x_ratio)/2)).asInteger;
		y_pos = (340*q.y_ratio).asInteger;
		x_size = (20*q.x_ratio).asInteger;
		x_pos=(100*q.x_ratio)+(i*temp)+(i*50*q.x_ratio)+(50*q.x_ratio);
		y_size = (250*q.min_effective_ratio).asInteger;
		q.reverb_slider[i]=Slider(w,Rect(x_pos, y_pos, x_size, y_size));
		q.reverb_slider[i].enabled=false;
	});


	q.num.do({arg i;
		var temp = (left_margin-(370*q.x_ratio))/2;
		y_pos = (605*q.y_ratio).asInteger;
		x_size = (30*q.x_ratio).asInteger;
		y_size = (15*q.min_effective_ratio).asInteger;
		x_pos = (100*q.x_ratio)+(i*temp)+(i*50*q.x_ratio)+(50*q.x_ratio)-(x_size/4);
		font_size = (15*q.min_effective_ratio).asInteger;
		reverb_slider_label[i] = StaticText(w, Rect(x_pos, y_pos, x_size, y_size));
		reverb_slider_label[i].align=\center;
		reverb_slider_label[i].font = Font("Calibri", font_size, bold:true);		
		reverb_slider_label[i].string = "Rev";
	});

	
	q.volume_image=Array.newClear(q.num);
	q.num.do({arg i;
		q.volume_image[i] = Array2D.new(4,4);
	});
	
	q.num.do({arg i;
		4.do({arg j;
			4.do({arg k;
				q.volume_image[i][j,k] = Image.open(catArgs(q.current_directory,"/figures/musician_",i,"_volume_",j,"_reverb_",k,".jpg" ));
			});
		}); 
	}); 



	q.volume_icon = Array.newClear(q.num);
	q.num.do({arg i;
		x_pos=(((i+1)*(left_margin/4))-((50*q.x_ratio)/2)).asInteger;
		y_pos = (625*q.y_ratio).asInteger;
		x_size = 50;
		y_size = 50;	
		q.volume_icon[i] = UserView(w, Rect(x_pos, y_pos, x_size, y_size));
   		q.volume_icon[i].drawFunc = { 
          	 	Pen.image(q.volume_image[i][0,0]); 
   		}; 
	});

	x_pos = 0;
	y_pos = (690*q.y_ratio).asInteger;
	x_size = left_margin;
	y_size = (40*q.y_ratio).asInteger;
	font_size = (20*q.min_effective_ratio).asInteger;
	volume_range_label = StaticText(w, Rect(x_pos, y_pos, x_size, y_size));
	volume_range_label.font = Font("Calibri", font_size);
	volume_range_label.align=\center;
	volume_range_label.string = "Sensitivity of Dynamic Volume";

	 
	x_pos = (45*q.x_ratio).asInteger;
	y_pos = (715*q.y_ratio).asInteger;
	x_size = left_margin-(2*(50*q.x_ratio));
	y_size = (30*q.y_ratio).asInteger;
	q.volume_range_slider = Slider(w, Rect(x_pos, y_pos, x_size, y_size));
	q.volume_range_slider.enabled=false;

	x_pos = (15*q.x_ratio).asInteger;
	y_pos = (715*q.y_ratio).asInteger;
	x_size = (30*q.x_ratio).asInteger;
	y_size = (30*q.y_ratio).asInteger;
	font_size = (30*q.min_effective_ratio).asInteger;
	volume_min_label = StaticText(w, Rect(x_pos, y_pos, x_size, y_size));
	volume_min_label.align=\left;
	volume_min_label.font = Font("Calibri", font_size, bold:true);		
	volume_min_label.string = "-";
		
	

	x_pos = left_margin-(45*q.x_ratio).asInteger;
	y_pos = (715*q.y_ratio).asInteger;
	x_size = (30*q.x_ratio).asInteger;
	y_size = (30*q.y_ratio).asInteger;
	font_size = (30*q.min_effective_ratio).asInteger;
	volume_max_label = StaticText(w, Rect(x_pos, y_pos, x_size, y_size));
	volume_max_label.align=\right;
	volume_max_label.font = Font("Calibri", font_size, bold:true);		
	volume_max_label.string = "+";	


	x_pos = 0;
	y_pos = (750*q.y_ratio).asInteger;
	x_size = left_margin;
	y_size = (40*q.y_ratio).asInteger;
	font_size = (20*q.min_effective_ratio).asInteger;
	reverb_range_label = StaticText(w, Rect(x_pos, y_pos, x_size, y_size));
	reverb_range_label.font = Font("Calibri", font_size);
	reverb_range_label.align=\center;
	reverb_range_label.string = "Sensitivity of Dynamic Reverb";

	 
	x_pos = (45*q.x_ratio).asInteger;
	y_pos = (775*q.y_ratio).asInteger;
	x_size = left_margin-(2*(50*q.x_ratio));
	y_size = (30*q.y_ratio).asInteger;
	q.reverb_range_slider = Slider(w, Rect(x_pos, y_pos, x_size, y_size));
	q.reverb_range_slider.enabled=false;

	x_pos = (15*q.x_ratio).asInteger;
	y_pos = (775*q.y_ratio).asInteger;
	x_size = (30*q.x_ratio).asInteger;
	y_size = (30*q.y_ratio).asInteger;
	font_size = (30*q.min_effective_ratio).asInteger;
	reverb_min_label = StaticText(w, Rect(x_pos, y_pos, x_size, y_size));
	reverb_min_label.align=\left;
	reverb_min_label.font = Font("Calibri", font_size, bold:true);		
	reverb_min_label.string = "-";
		
	

	x_pos = left_margin-(45*q.x_ratio).asInteger;
	y_pos = (775*q.y_ratio).asInteger;
	x_size = (30*q.x_ratio).asInteger;
	y_size = (30*q.y_ratio).asInteger;
	font_size = (30*q.min_effective_ratio).asInteger;
	reverb_max_label = StaticText(w, Rect(x_pos, y_pos, x_size, y_size));
	reverb_max_label.align=\right;
	reverb_max_label.font = Font("Calibri", font_size, bold:true);		
	reverb_max_label.string = "+";		

	


	//now we set all values related to the viewer
	q.viewer_total_width = q.x_available-left_margin-25;
	q.viewer_total_height = q.y_available-50;

	c = UserView(w, Rect(left_margin,25,q.viewer_total_width,q.viewer_total_height));
	c.background_(Color.gray(0.8));

	


	q.calibrate_button.action = ({|view|	
			if(view.value==0, {
				q.calibrate_label.string = "Please move around your space while we collect samples.";
				q.kinect_calibration_part_1();
			});		
	});

	q.jmess_cmd = catArgs("jmess -D -c ", q.current_directory,"/jmess_files/jmess_with_ardour.xml");

	q.connect_button.action=({|view|
		if(view.value==1, {	
			q.jmess_cmd.unixCmd{|res2,pid2|	
				q.startjam();
			};
		},{
			"Stopping system!".postln;
			q.stopjam();
		});
	});



	w.front;
	

	w.onClose={
		if(q.connected,{
			q.stopjam();
		});
	};
	
	q.player_labels=Array.newClear(q.num);
	q.num.do({arg i;
		q.player_labels[i]=StaticText(w);
		q.player_labels[i].font=Font("Calibri",(20*q.min_effective_ratio).asInteger);
		q.player_labels[i].stringColor=Color.black;
		q.player_labels[i].align=\center;
		if(i==q.player_num,{
			q.player_labels[i].string = "";
		},{
			q.player_labels[i].string = i;
		});
	});	


	"now we set up the room parameters".postln;
	q.kinect_parameters_part_1();
	
};


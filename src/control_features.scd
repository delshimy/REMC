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

q.createGUI_controls = { |q|
	var position_slider = Array.newClear(2);
	var musician_label = Array.newClear(2);
	var positions = Array2D.new(2,2);
	var features_question, dvm_box, dvm_label, esp_box, esp_label, rms_box, rms_label;

	b = Window("Sliders to simulate remote musicians", Rect(0, 0, 1450, 550));
	
	//this code is based on the assumption that Musician 0 is always the local musician, who controls Musicians 1 and 2.


	2.do({arg i;
		2.do({arg j;
			positions[i,j]=0;
		});
	});

	

	musician_label[0] = StaticText(b, Rect(500,0,400,100));
	musician_label[0].string = catArgs("Musician 1"); 	
	musician_label[0].font = Font("Calibri",40);
	musician_label[0].align=\center;

	musician_label[1] = StaticText(b, Rect(1000,0,400,100));
	musician_label[1].string = catArgs("Musician 2"); 	
	musician_label[1].font = Font("Calibri",40);
	musician_label[1].align=\center;




	position_slider[0] = Slider2D(b, Rect(500, 100, 400, 400));
	position_slider[0].x_(0); // initial location of x
	position_slider[0].y_(0);   // initial location of y
	position_slider[0].background_(Color.green(0.4));


	position_slider[1] = Slider2D(b, Rect(1000, 100, 400, 400));
	position_slider[1].x_(0); // initial location of x
	position_slider[1].y_(0);   // initial location of y
	position_slider[1].background_(Color.blue(0.4));

	2.do({arg i;
		position_slider[i].action_({|sl|
			positions[i,0]=(sl.x*(q.kinect_max_x[i+1]-q.kinect_min_x[i+1]))+q.kinect_min_x[i+1];
			positions[i,1]=(sl.y*(q.kinect_max_y[i+1]-q.kinect_min_y[i+1]))+q.kinect_min_y[i+1];
	

			NetAddr("localhost",57120).sendMsg("/client", "torso", (i+1), positions[i,0],0,positions[i,1],0,0,0);
			NetAddr("localhost",57121).sendMsg("/client", "torso", (i+1), positions[i,0],0,positions[i,1],0,0,0);	
			NetAddr("localhost",57120).sendMsg("/client", "head", (i+1), positions[i,0],0,positions[i,1],0,0,0);
			NetAddr("localhost",57121).sendMsg("/client", "head", (i+1), positions[i,0],0,positions[i,1],0,0,0);
		});
	});
	
	b.front;


	features_question=StaticText(b, Rect(0,100,500,100));
	features_question.align=\center;
	features_question.font=Font("Calibri",25);
	features_question.string = ("Select performance features:");
	//features_question.stringColor=Color.gray(0.4);

	
	dvm_box = CheckBox(b, Rect(100,200,30,30));
	dvm_box.value = 1;
	dvm_box.action = {arg value;
		if (value == 1,{
			q.dvm = true;
			q.cond_dvm = "dvm_on";
		},{
			q.dvm = false;
			q.cond_dvm = "dvm_off"
		});	
	};
		
	dvm_label = StaticText(b, Rect(130,200,300,30));
	dvm_label.string = "Dynamic Volume Mixing"; 	
	dvm_label.font = Font("Calibri",20);
	dvm_label.stringColor=Color.gray(0.4);

	
	esp_box = CheckBox(b, Rect(100,250,30,30));
	esp_box.value = 1;
	esp_box.action = {arg value;
		if (value == 1,{
			q.esp = true;
			q.cond_esp = "esp_on";
		},{
			q.esp = false;
			q.cond_esp = "esp_off";
		});	
	};

	esp_label = StaticText(b, Rect(130,250,300,30));
	esp_label.string = "Enhanced Stereo Panning"; 	
	esp_label.font = Font("Calibri",20);
	esp_label.stringColor=Color.gray(0.4);

	
	rms_box = CheckBox(b, Rect(100,300,30,30));
	rms_box.value = 1;
	rms_box.action = {arg value;
		if (value == 1,{
			q.rms = true;
			q.cond_rms = "rms_on";
		},{
			q.rms = false;
			q.cond_rms = "rms_off";
		});	
	};


	rms_label = StaticText(b, Rect(130,300,350,30));
	rms_label.string = "Remote Musician Spatialization"; 	
	rms_label.font = Font("Calibri",20);
	rms_label.stringColor=Color.gray(0.4);

};

q.createGUI_controls;

)





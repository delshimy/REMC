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


q.createGUI_controls = { |q|
	var position_slider = Array.newClear(2);
	var musician_label = Array.newClear(2);
	var positions = Array2D.new(2,2);
	var features_question, dv_box, dv_label, tp_box, tp_label, ms_box, ms_label, mc_label, mc_box, dr_box, dr_label;

	b = Window("Reactive Environment for Network Music Performance - Controls ", Rect(0, 0, 1200, 500));
	
	//this code is based on the assumption that Musician 0 is always the local musician, who controls Musicians 1 and 2.


	2.do({arg i;
		2.do({arg j;
			positions[i,j]=0;
		});
	});

	

	musician_label[0] = StaticText(b, Rect(425,0,350,100));
	musician_label[0].string = catArgs("Musician 1"); 	
	musician_label[0].font = Font("Calibri",40);
	musician_label[0].align=\center;

	musician_label[1] = StaticText(b, Rect(825,0,350,100));
	musician_label[1].string = catArgs("Musician 2"); 	
	musician_label[1].font = Font("Calibri",40);
	musician_label[1].align=\center;




	position_slider[0] = Slider2D(b, Rect(425, 100, 350, 350));
	position_slider[0].x_(0); // initial location of x
	position_slider[0].y_(0);   // initial location of y
	position_slider[0].background_(Color.green(0.4));


	position_slider[1] = Slider2D(b, Rect(825, 100, 350, 350));
	position_slider[1].x_(0); // initial location of x
	position_slider[1].y_(0);   // initial location of y
	position_slider[1].background_(Color.blue(0.4));

	2.do({arg i;
		position_slider[i].action_({|sl|
			positions[i,0]=(sl.x*(q.kinect_max_x_ideal-q.kinect_min_x_ideal))+q.kinect_min_x_ideal;
			positions[i,1]=((1-sl.y)*(q.kinect_max_y_ideal-q.kinect_min_y_ideal))+q.kinect_min_y_ideal;


			NetAddr("localhost",57120).sendMsg("/client", "torso", (i+1), positions[i,0],0,positions[i,1],0,0,0);
			NetAddr("localhost",57121).sendMsg("/client", "torso", (i+1), positions[i,0],0,positions[i,1],0,0,0);	
			NetAddr("localhost",57120).sendMsg("/client", "head", (i+1), positions[i,0],0,positions[i,1],0,0,0);
			NetAddr("localhost",57121).sendMsg("/client", "head", (i+1), positions[i,0],0,positions[i,1],0,0,0);
		});
	});
	
	b.front;


	features_question=StaticText(b, Rect(0,60,400,100));
	features_question.align=\center;
	features_question.font=Font("Calibri",25);
	features_question.string = ("Select performance features:");
	//features_question.stringColor=Color.gray(0.4);

	
	dv_box = CheckBox(b, Rect(50,160,30,30));
	dv_box.action = {arg value;
		if (dv_box.value,{
			q.dv = true;
			q.cond_dv = "dv_on";
			q.volume_range_slider.enabled = true;
			q.volume_range_slider.value=0.5;
		},{
			q.dv = false;
			q.cond_dv = "dv_off";
			q.volume_range_slider.enabled = false;
		});	
	};

	if(q.dv != nil, {
		dv_box.value = q.dv;
	},{
		dv_box.value = true;
	});
		
	dv_label = StaticText(b, Rect(80,160,295,30));
	dv_label.string = "Dynamic Volume"; 	
	dv_label.font = Font("Calibri",20);
	dv_label.stringColor=Color.gray(0.4);
	

	dr_box = CheckBox(b, Rect(50,210,30,30));
	dr_box.action = {
		if (dr_box.value,{
			q.dr = true;
			q.cond_dr = "dr_on";
			q.reverb_range_slider.enabled = true;
			q.reverb_range_slider.value=0.5;
		},{
			q.dr = false;
			q.cond_dr = "dr_off";
			q.reverb_range_slider.enabled = false;
		});	
	};
		
		
	if(q.dr != nil, {
		dr_box.value = q.dr;
	},{
		dr_box.value = true;
	});

	dr_label = StaticText(b, Rect(80,210,295,30));
	dr_label.string = "Dynamic Reverb"; 	
	dr_label.font = Font("Calibri",20);
	dr_label.stringColor=Color.gray(0.4);



	tp_box = CheckBox(b, Rect(50,260,30,30));
	tp_box.action = {
		if (tp_box.value,{
			q.tp = true;
			q.cond_tp = "tp_on";
			q.ms=false;
			q.cond_ms="ms_off";
			ms_box.value=false;
			q.mc=false;
			q.cond_mc="mc_off";
			mc_box.value=false;
		},{
			q.tp = false;
			q.cond_tp = "tp_off";
		});	
	};

	if(q.tp != nil, {
		tp_box.value = q.tp;
	},{
		tp_box.value = true;
	});


	tp_label = StaticText(b, Rect(80,260,295,30));
	tp_label.string = "Track Panning"; 	
	tp_label.font = Font("Calibri",20);
	tp_label.stringColor=Color.gray(0.4);

	
	mc_box = CheckBox(b, Rect(50,310,30,30));
	mc_box.action = {
		if (mc_box.value,{
			q.mc = true;
			q.cond_mc = "mc_on";
			q.tp=false;
			q.cond_tp="tp_off";
			tp_box.value=false;
			q.ms=false;
			q.cond_ms="ms_off";
			ms_box.value=false;
		},{
			q.mc = false;
			q.cond_mc = "mc_off";
		});	
	};

	if(q.mc != nil, {
		mc_box.value = q.mc;
	},{
		mc_box.value = true;
	});


	mc_label = StaticText(b, Rect(80,310,295,30));
	mc_label.string = "Mix Control"; 	
	mc_label.font = Font("Calibri",20);
	mc_label.stringColor=Color.gray(0.4);

	


	ms_box = CheckBox(b, Rect(50,360,30,30));
	ms_box.action = {
		if (ms_box.value,{
			q.ms = true;
			q.cond_ms = "ms_on";
			q.tp=false;
			q.cond_tp="tp_off";
			tp_box.value=false;
			q.mc=false;
			q.cond_mc="mc_off";
			mc_box.value=false;
		},{
			q.ms = false;
			q.cond_ms = "ms_off";
		});	
	};

	if(q.ms != nil, {
		ms_box.value = q.ms;
	},{
		ms_box.value = true;
	});	


	ms_label = StaticText(b, Rect(80,360,325,30));
	ms_label.string = "Musician Spatialization"; 	
	ms_label.font = Font("Calibri",20);
	ms_label.stringColor=Color.gray(0.4);


	

};

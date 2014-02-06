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

SynthDef(\player_mix,{ |amp_instrument=#[0,0,0], channel_out, roomsize, revtime_instrument=#[0,0,0], damp_instrument=#[0,0,0]|	 
			var source_instruments=Array.newClear(q.num);
			//var source_vocals=Array.newClear(q.num);
			var source_instruments_plus_reverb = Array.newClear(q.num);
			//var source_vocals_plus_reverb = Array.newClear(q.num);
			var reverb_instruments = Array.newClear(q.num);
			//var reverb_vocals = Array.newClear(q.num);
			var source_mix;	
			//var in_channels=Array.newClear(q.num*2);
			var in_channels=Array.newClear(q.num);
			//format of GVerb is:
			//GVerb.ar(source, roomsize, revtime, damping,inputbw, spread, drylevel.dbamp,earlylevel.dbamp, taillevel.dbamp, roomsize, 0.3)							
			(q.num).do({arg i;
				in_channels[i]=i;
				//in_channels[i+q.num]=i+q.num;
				source_instruments[i]=Limiter.ar(SoundIn.ar(in_channels[i],amp_instrument[i]), amp_instrument[i], 0.01);
				//source_vocals[i]=SoundIn.ar(in_channels[i+q.num],amp_vocals[i]);
				//reverb_instruments[i]=GVerb.ar(source_instruments[i], roomsize, revtime_instrument[i], damp_instrument[i],inputbw,spread,drylevel.dbamp, earlylevel.dbamp,taillevel.dbamp, roomsize, 0.3); 
				//settings for church are [\roomsize, 80, \revtime, 4.85, \damping, 0.41, \inputbw, 0.19, \drylevel -3, \earlylevel, -9, \taillevel, -11]
				reverb_instruments[i]= GVerb.ar(source_instruments[i], 16, revtime_instrument[i], damp_instrument[i], 0.19, 15, (-3).dbamp, (-15).dbamp, (-17).dbamp, 16, 0.3); 
				//reverb_vocals[i]= GVerb.ar(source_vocals[i], 16, reverb_time[i], 0.10, 0.95, 15, (-3).dbamp, (-15).dbamp, (-17).dbamp, 16, 0.3); 
				source_instruments_plus_reverb[i]=source_instruments[i]+reverb_instruments[i];
				//source_vocals_plus_reverb[i]=source_vocals[i]+reverb_vocals[i];
			});		
			//source_mix=((source_vocals_plus_reverb.sum)+(source_instruments_plus_reverb.sum))/(q.num*2);
			source_mix=(source_instruments_plus_reverb.sum)/(q.num);
			//source_mix=(source_instruments.sum)/(q.num);
			Out.ar(channel_out,source_mix);
}).add;




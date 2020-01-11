package frc.paths;

import com.team319.trajectory.Path;

public class Dummy extends Path {
   // dt,x,y,left.pos,left.vel,left.acc,left.jerk,center.pos,center.vel,center.acc,center.jerk,right.pos,right.vel,right.acc,right.jerk,heading
	private static final double[][] points = {
				{0.0200,1.0004,2.0000,0.0004,0.0200,1.0000,0.0000,0.0004,0.0200,1.0000,0.0000,0.0004,0.0200,1.0000,0.0000,0.0000},
				{0.0200,1.0012,2.0000,0.0012,0.0400,1.0000,0.0000,0.0012,0.0400,1.0000,0.0000,0.0012,0.0400,1.0000,0.0000,0.0000},
				{0.0200,1.0024,2.0000,0.0024,0.0600,1.0000,-0.0000,0.0024,0.0600,1.0000,0.0000,0.0024,0.0600,1.0000,-0.0000,0.0000},
				{0.0200,1.0040,2.0000,0.0040,0.0800,1.0000,0.0000,0.0040,0.0800,1.0000,0.0000,0.0040,0.0800,1.0000,0.0000,0.0000},
				{0.0200,1.0060,2.0000,0.0060,0.1000,1.0000,-0.0000,0.0060,0.1000,1.0000,0.0000,0.0060,0.1000,1.0000,-0.0000,0.0000},
				{0.0200,1.0084,2.0000,0.0084,0.1200,1.0000,0.0000,0.0084,0.1200,1.0000,0.0000,0.0084,0.1200,1.0000,0.0000,0.0000},
				{0.0200,1.0112,2.0000,0.0112,0.1400,1.0000,0.0000,0.0112,0.1400,1.0000,0.0000,0.0112,0.1400,1.0000,0.0000,0.0000},
				{0.0200,1.0144,2.0000,0.0144,0.1600,1.0000,0.0000,0.0144,0.1600,1.0000,0.0000,0.0144,0.1600,1.0000,0.0000,0.0000},
				{0.0200,1.0180,2.0000,0.0180,0.1800,1.0000,-0.0000,0.0180,0.1800,1.0000,0.0000,0.0180,0.1800,1.0000,-0.0000,0.0000},
				{0.0200,1.0220,2.0000,0.0220,0.2000,1.0000,0.0000,0.0220,0.2000,1.0000,0.0000,0.0220,0.2000,1.0000,0.0000,0.0000},
				{0.0200,1.0264,2.0000,0.0264,0.2200,1.0000,-0.0000,0.0264,0.2200,1.0000,0.0000,0.0264,0.2200,1.0000,-0.0000,0.0000},
				{0.0200,1.0312,2.0000,0.0312,0.2400,1.0000,0.0000,0.0312,0.2400,1.0000,0.0000,0.0312,0.2400,1.0000,0.0000,0.0000},
				{0.0200,1.0364,2.0000,0.0364,0.2600,1.0000,0.0000,0.0364,0.2600,1.0000,0.0000,0.0364,0.2600,1.0000,0.0000,0.0000},
				{0.0200,1.0420,2.0000,0.0420,0.2800,1.0000,-0.0000,0.0420,0.2800,1.0000,0.0000,0.0420,0.2800,1.0000,-0.0000,0.0000},
				{0.0200,1.0480,2.0000,0.0480,0.3000,1.0000,0.0000,0.0480,0.3000,1.0000,0.0000,0.0480,0.3000,1.0000,0.0000,0.0000},
				{0.0200,1.0544,2.0000,0.0544,0.3200,1.0000,0.0000,0.0544,0.3200,1.0000,0.0000,0.0544,0.3200,1.0000,0.0000,0.0000},
				{0.0200,1.0612,2.0000,0.0612,0.3400,1.0000,0.0000,0.0612,0.3400,1.0000,0.0000,0.0612,0.3400,1.0000,0.0000,0.0000},
				{0.0200,1.0684,2.0000,0.0684,0.3600,1.0000,-0.0000,0.0684,0.3600,1.0000,0.0000,0.0684,0.3600,1.0000,-0.0000,0.0000},
				{0.0200,1.0760,2.0000,0.0760,0.3800,1.0000,0.0000,0.0760,0.3800,1.0000,0.0000,0.0760,0.3800,1.0000,0.0000,0.0000},
				{0.0200,1.0840,2.0000,0.0840,0.4000,1.0000,-0.0000,0.0840,0.4000,1.0000,0.0000,0.0840,0.4000,1.0000,-0.0000,0.0000},
				{0.0200,1.0924,2.0000,0.0924,0.4200,1.0000,0.0000,0.0924,0.4200,1.0000,0.0000,0.0924,0.4200,1.0000,0.0000,0.0000},
				{0.0200,1.1012,2.0000,0.1012,0.4400,1.0000,-0.0000,0.1012,0.4400,1.0000,0.0000,0.1012,0.4400,1.0000,-0.0000,0.0000},
				{0.0200,1.1104,2.0000,0.1104,0.4600,1.0000,0.0000,0.1104,0.4600,1.0000,0.0000,0.1104,0.4600,1.0000,0.0000,0.0000},
				{0.0200,1.1200,2.0000,0.1200,0.4800,1.0000,0.0000,0.1200,0.4800,1.0000,0.0000,0.1200,0.4800,1.0000,0.0000,0.0000},
				{0.0200,1.1300,2.0000,0.1300,0.5000,1.0000,-0.0000,0.1300,0.5000,1.0000,0.0000,0.1300,0.5000,1.0000,-0.0000,0.0000},
				{0.0200,1.1404,2.0000,0.1404,0.5200,1.0000,-0.0000,0.1404,0.5200,1.0000,0.0000,0.1404,0.5200,1.0000,-0.0000,0.0000},
				{0.0200,1.1512,2.0000,0.1512,0.5400,1.0000,0.0000,0.1512,0.5400,1.0000,0.0000,0.1512,0.5400,1.0000,0.0000,0.0000},
				{0.0200,1.1624,2.0000,0.1624,0.5600,1.0000,-0.0000,0.1624,0.5600,1.0000,0.0000,0.1624,0.5600,1.0000,-0.0000,0.0000},
				{0.0200,1.1740,2.0000,0.1740,0.5800,1.0000,0.0000,0.1740,0.5800,1.0000,0.0000,0.1740,0.5800,1.0000,0.0000,0.0000},
				{0.0200,1.1860,2.0000,0.1860,0.6000,1.0000,-0.0000,0.1860,0.6000,1.0000,0.0000,0.1860,0.6000,1.0000,-0.0000,0.0000},
				{0.0200,1.1984,2.0000,0.1984,0.6200,1.0000,0.0000,0.1984,0.6200,1.0000,0.0000,0.1984,0.6200,1.0000,0.0000,0.0000},
				{0.0200,1.2112,2.0000,0.2112,0.6400,1.0000,0.0000,0.2112,0.6400,1.0000,0.0000,0.2112,0.6400,1.0000,0.0000,0.0000},
				{0.0200,1.2244,2.0000,0.2244,0.6600,1.0000,-0.0000,0.2244,0.6600,1.0000,0.0000,0.2244,0.6600,1.0000,-0.0000,0.0000},
				{0.0200,1.2380,2.0000,0.2380,0.6800,1.0000,0.0000,0.2380,0.6800,1.0000,0.0000,0.2380,0.6800,1.0000,0.0000,0.0000},
				{0.0200,1.2520,2.0000,0.2520,0.7000,1.0000,0.0000,0.2520,0.7000,1.0000,0.0000,0.2520,0.7000,1.0000,0.0000,0.0000},
				{0.0200,1.2664,2.0000,0.2664,0.7200,1.0000,0.0000,0.2664,0.7200,1.0000,0.0000,0.2664,0.7200,1.0000,0.0000,0.0000},
				{0.0200,1.2812,2.0000,0.2812,0.7400,1.0000,-0.0000,0.2812,0.7400,1.0000,0.0000,0.2812,0.7400,1.0000,-0.0000,0.0000},
				{0.0200,1.2964,2.0000,0.2964,0.7600,1.0000,0.0000,0.2964,0.7600,1.0000,0.0000,0.2964,0.7600,1.0000,0.0000,0.0000},
				{0.0200,1.3120,2.0000,0.3120,0.7800,1.0000,0.0000,0.3120,0.7800,1.0000,0.0000,0.3120,0.7800,1.0000,0.0000,0.0000},
				{0.0200,1.3280,2.0000,0.3280,0.8000,1.0000,0.0000,0.3280,0.8000,1.0000,0.0000,0.3280,0.8000,1.0000,0.0000,0.0000},
				{0.0200,1.3444,2.0000,0.3444,0.8200,1.0000,0.0000,0.3444,0.8200,1.0000,0.0000,0.3444,0.8200,1.0000,0.0000,0.0000},
				{0.0200,1.3612,2.0000,0.3612,0.8400,1.0000,0.0000,0.3612,0.8400,1.0000,0.0000,0.3612,0.8400,1.0000,0.0000,0.0000},
				{0.0200,1.3784,2.0000,0.3784,0.8600,1.0000,-0.0000,0.3784,0.8600,1.0000,0.0000,0.3784,0.8600,1.0000,-0.0000,0.0000},
				{0.0200,1.3960,2.0000,0.3960,0.8800,1.0000,0.0000,0.3960,0.8800,1.0000,0.0000,0.3960,0.8800,1.0000,0.0000,0.0000},
				{0.0200,1.4140,2.0000,0.4140,0.9000,1.0000,-0.0000,0.4140,0.9000,1.0000,0.0000,0.4140,0.9000,1.0000,-0.0000,0.0000},
				{0.0200,1.4324,2.0000,0.4324,0.9200,1.0000,0.0000,0.4324,0.9200,1.0000,0.0000,0.4324,0.9200,1.0000,0.0000,0.0000},
				{0.0200,1.4512,2.0000,0.4512,0.9400,1.0000,-0.0000,0.4512,0.9400,1.0000,0.0000,0.4512,0.9400,1.0000,-0.0000,0.0000},
				{0.0200,1.4704,2.0000,0.4704,0.9600,1.0000,0.0000,0.4704,0.9600,1.0000,0.0000,0.4704,0.9600,1.0000,0.0000,0.0000},
				{0.0200,1.4900,2.0000,0.4900,0.9800,1.0000,0.0000,0.4900,0.9800,1.0000,0.0000,0.4900,0.9800,1.0000,0.0000,0.0000},
				{0.0200,1.5100,2.0000,0.5100,1.0000,1.0000,-0.0000,0.5100,1.0000,1.0000,0.0000,0.5100,1.0000,1.0000,-0.0000,0.0000},
				{0.0200,1.5304,2.0000,0.5304,1.0200,1.0000,-0.0000,0.5304,1.0200,1.0000,0.0000,0.5304,1.0200,1.0000,-0.0000,0.0000},
				{0.0200,1.5512,2.0000,0.5512,1.0400,1.0000,0.0000,0.5512,1.0400,1.0000,0.0000,0.5512,1.0400,1.0000,0.0000,0.0000},
				{0.0200,1.5724,2.0000,0.5724,1.0600,1.0000,-0.0000,0.5724,1.0600,1.0000,0.0000,0.5724,1.0600,1.0000,-0.0000,0.0000},
				{0.0200,1.5940,2.0000,0.5940,1.0800,1.0000,0.0000,0.5940,1.0800,1.0000,0.0000,0.5940,1.0800,1.0000,0.0000,0.0000},
				{0.0200,1.6160,2.0000,0.6160,1.1000,1.0000,0.0000,0.6160,1.1000,1.0000,0.0000,0.6160,1.1000,1.0000,0.0000,0.0000},
				{0.0200,1.6384,2.0000,0.6384,1.1200,1.0000,-0.0000,0.6384,1.1200,1.0000,0.0000,0.6384,1.1200,1.0000,-0.0000,0.0000},
				{0.0200,1.6612,2.0000,0.6612,1.1400,1.0000,0.0000,0.6612,1.1400,1.0000,0.0000,0.6612,1.1400,1.0000,0.0000,0.0000},
				{0.0200,1.6844,2.0000,0.6844,1.1600,1.0000,-0.0000,0.6844,1.1600,1.0000,0.0000,0.6844,1.1600,1.0000,-0.0000,0.0000},
				{0.0200,1.7080,2.0000,0.7080,1.1800,1.0000,0.0000,0.7080,1.1800,1.0000,0.0000,0.7080,1.1800,1.0000,0.0000,0.0000},
				{0.0200,1.7320,2.0000,0.7320,1.2000,1.0000,-0.0000,0.7320,1.2000,1.0000,0.0000,0.7320,1.2000,1.0000,-0.0000,0.0000},
				{0.0200,1.7564,2.0000,0.7564,1.2200,1.0000,0.0000,0.7564,1.2200,1.0000,0.0000,0.7564,1.2200,1.0000,0.0000,0.0000},
				{0.0200,1.7812,2.0000,0.7812,1.2400,1.0000,0.0000,0.7812,1.2400,1.0000,0.0000,0.7812,1.2400,1.0000,0.0000,0.0000},
				{0.0200,1.8064,2.0000,0.8064,1.2600,1.0000,-0.0000,0.8064,1.2600,1.0000,0.0000,0.8064,1.2600,1.0000,-0.0000,0.0000},
				{0.0200,1.8320,2.0000,0.8320,1.2800,1.0000,0.0000,0.8320,1.2800,1.0000,0.0000,0.8320,1.2800,1.0000,0.0000,0.0000},
				{0.0200,1.8580,2.0000,0.8580,1.3000,1.0000,-0.0000,0.8580,1.3000,1.0000,0.0000,0.8580,1.3000,1.0000,-0.0000,0.0000},
				{0.0200,1.8844,2.0000,0.8844,1.3200,1.0000,0.0000,0.8844,1.3200,1.0000,0.0000,0.8844,1.3200,1.0000,0.0000,0.0000},
				{0.0200,1.9112,2.0000,0.9112,1.3400,1.0000,0.0000,0.9112,1.3400,1.0000,0.0000,0.9112,1.3400,1.0000,0.0000,0.0000},
				{0.0200,1.9384,2.0000,0.9384,1.3600,1.0000,0.0000,0.9384,1.3600,1.0000,0.0000,0.9384,1.3600,1.0000,0.0000,0.0000},
				{0.0200,1.9660,2.0000,0.9660,1.3800,1.0000,-0.0000,0.9660,1.3800,1.0000,0.0000,0.9660,1.3800,1.0000,-0.0000,0.0000},
				{0.0200,1.9940,2.0000,0.9940,1.4000,1.0000,0.0000,0.9940,1.4000,1.0000,0.0000,0.9940,1.4000,1.0000,0.0000,0.0000},
				{0.0200,2.0224,2.0000,1.0224,1.4200,1.0000,-0.0000,1.0224,1.4200,1.0000,0.0000,1.0224,1.4200,1.0000,-0.0000,0.0000},
				{0.0200,2.0512,2.0000,1.0512,1.4400,1.0000,0.0000,1.0512,1.4400,1.0000,0.0000,1.0512,1.4400,1.0000,0.0000,0.0000},
				{0.0200,2.0804,2.0000,1.0804,1.4600,1.0000,0.0000,1.0804,1.4600,1.0000,0.0000,1.0804,1.4600,1.0000,0.0000,0.0000},
				{0.0200,2.1100,2.0000,1.1100,1.4800,1.0000,0.0000,1.1100,1.4800,1.0000,0.0000,1.1100,1.4800,1.0000,0.0000,0.0000},
				{0.0200,2.1400,2.0000,1.1400,1.5000,1.0000,-0.0000,1.1400,1.5000,1.0000,0.0000,1.1400,1.5000,1.0000,-0.0000,0.0000},
				{0.0200,2.1704,2.0000,1.1704,1.5200,1.0000,0.0000,1.1704,1.5200,1.0000,0.0000,1.1704,1.5200,1.0000,0.0000,0.0000},
				{0.0200,2.2012,2.0000,1.2012,1.5400,1.0000,-0.0000,1.2012,1.5400,1.0000,0.0000,1.2012,1.5400,1.0000,-0.0000,0.0000},
				{0.0200,2.2324,2.0000,1.2324,1.5600,1.0000,0.0000,1.2324,1.5600,1.0000,0.0000,1.2324,1.5600,1.0000,0.0000,0.0000},
				{0.0200,2.2640,2.0000,1.2640,1.5800,1.0000,-0.0000,1.2640,1.5800,1.0000,0.0000,1.2640,1.5800,1.0000,-0.0000,0.0000},
				{0.0200,2.2960,2.0000,1.2960,1.6000,1.0000,0.0000,1.2960,1.6000,1.0000,0.0000,1.2960,1.6000,1.0000,0.0000,0.0000},
				{0.0200,2.3284,2.0000,1.3284,1.6200,1.0000,0.0000,1.3284,1.6200,1.0000,0.0000,1.3284,1.6200,1.0000,0.0000,0.0000},
				{0.0200,2.3612,2.0000,1.3612,1.6400,1.0000,0.0000,1.3612,1.6400,1.0000,0.0000,1.3612,1.6400,1.0000,0.0000,0.0000},
				{0.0200,2.3944,2.0000,1.3944,1.6600,1.0000,0.0000,1.3944,1.6600,1.0000,0.0000,1.3944,1.6600,1.0000,0.0000,0.0000},
				{0.0200,2.4280,2.0000,1.4280,1.6800,1.0000,-0.0000,1.4280,1.6800,1.0000,0.0000,1.4280,1.6800,1.0000,-0.0000,0.0000},
				{0.0200,2.4620,2.0000,1.4620,1.7000,1.0000,0.0000,1.4620,1.7000,1.0000,0.0000,1.4620,1.7000,1.0000,0.0000,0.0000},
				{0.0200,2.4964,2.0000,1.4964,1.7200,1.0000,0.0000,1.4964,1.7200,1.0000,0.0000,1.4964,1.7200,1.0000,0.0000,0.0000},
				{0.0200,2.5312,2.0000,1.5312,1.7400,1.0000,-0.0000,1.5312,1.7400,1.0000,0.0000,1.5312,1.7400,1.0000,-0.0000,0.0000},
				{0.0200,2.5664,2.0000,1.5664,1.7600,1.0000,0.0000,1.5664,1.7600,1.0000,0.0000,1.5664,1.7600,1.0000,0.0000,0.0000},
				{0.0200,2.6020,2.0000,1.6020,1.7800,1.0000,-0.0000,1.6020,1.7800,1.0000,0.0000,1.6020,1.7800,1.0000,-0.0000,0.0000},
				{0.0200,2.6380,2.0000,1.6380,1.8000,1.0000,0.0000,1.6380,1.8000,1.0000,0.0000,1.6380,1.8000,1.0000,0.0000,0.0000},
				{0.0200,2.6744,2.0000,1.6744,1.8200,1.0000,0.0000,1.6744,1.8200,1.0000,0.0000,1.6744,1.8200,1.0000,0.0000,0.0000},
				{0.0200,2.7112,2.0000,1.7112,1.8400,1.0000,-0.0000,1.7112,1.8400,1.0000,0.0000,1.7112,1.8400,1.0000,-0.0000,0.0000},
				{0.0200,2.7484,2.0000,1.7484,1.8600,1.0000,0.0000,1.7484,1.8600,1.0000,0.0000,1.7484,1.8600,1.0000,0.0000,0.0000},
				{0.0200,2.7860,2.0000,1.7860,1.8800,1.0000,0.0000,1.7860,1.8800,1.0000,0.0000,1.7860,1.8800,1.0000,0.0000,0.0000},
				{0.0200,2.8240,2.0000,1.8240,1.9000,1.0000,-0.0000,1.8240,1.9000,1.0000,0.0000,1.8240,1.9000,1.0000,-0.0000,0.0000},
				{0.0200,2.8624,2.0000,1.8624,1.9200,1.0000,0.0000,1.8624,1.9200,1.0000,0.0000,1.8624,1.9200,1.0000,0.0000,0.0000},
				{0.0200,2.9012,2.0000,1.9012,1.9400,1.0000,0.0000,1.9012,1.9400,1.0000,0.0000,1.9012,1.9400,1.0000,0.0000,0.0000},
				{0.0200,2.9404,2.0000,1.9404,1.9600,1.0000,0.0000,1.9404,1.9600,1.0000,0.0000,1.9404,1.9600,1.0000,0.0000,0.0000},
				{0.0200,2.9800,2.0000,1.9800,1.9800,1.0000,0.0000,1.9800,1.9800,1.0000,0.0000,1.9800,1.9800,1.0000,0.0000,0.0000},
				{0.0200,3.0200,2.0000,2.0200,2.0000,1.0000,0.0000,2.0200,2.0000,1.0000,0.0000,2.0200,2.0000,1.0000,0.0000,0.0000},
				{0.0200,3.0596,2.0000,2.0596,1.9800,-1.0000,-100.0000,2.0596,1.9800,-1.0000,0.0000,2.0596,1.9800,-1.0000,-100.0000,0.0000},
				{0.0200,3.0988,2.0000,2.0988,1.9600,-1.0000,-0.0000,2.0988,1.9600,-1.0000,0.0000,2.0988,1.9600,-1.0000,-0.0000,0.0000},
				{0.0200,3.1376,2.0000,2.1376,1.9400,-1.0000,-0.0000,2.1376,1.9400,-1.0000,0.0000,2.1376,1.9400,-1.0000,-0.0000,0.0000},
				{0.0200,3.1760,2.0000,2.1760,1.9200,-1.0000,0.0000,2.1760,1.9200,-1.0000,0.0000,2.1760,1.9200,-1.0000,0.0000,0.0000},
				{0.0200,3.2140,2.0000,2.2140,1.9000,-1.0000,-0.0000,2.2140,1.9000,-1.0000,0.0000,2.2140,1.9000,-1.0000,-0.0000,0.0000},
				{0.0200,3.2516,2.0000,2.2516,1.8800,-1.0000,0.0000,2.2516,1.8800,-1.0000,0.0000,2.2516,1.8800,-1.0000,0.0000,0.0000},
				{0.0200,3.2888,2.0000,2.2888,1.8600,-1.0000,0.0000,2.2888,1.8600,-1.0000,0.0000,2.2888,1.8600,-1.0000,0.0000,0.0000},
				{0.0200,3.3256,2.0000,2.3256,1.8400,-1.0000,-0.0000,2.3256,1.8400,-1.0000,0.0000,2.3256,1.8400,-1.0000,-0.0000,0.0000},
				{0.0200,3.3620,2.0000,2.3620,1.8200,-1.0000,0.0000,2.3620,1.8200,-1.0000,0.0000,2.3620,1.8200,-1.0000,0.0000,0.0000},
				{0.0200,3.3980,2.0000,2.3980,1.8000,-1.0000,0.0000,2.3980,1.8000,-1.0000,0.0000,2.3980,1.8000,-1.0000,0.0000,0.0000},
				{0.0200,3.4336,2.0000,2.4336,1.7800,-1.0000,0.0000,2.4336,1.7800,-1.0000,0.0000,2.4336,1.7800,-1.0000,0.0000,0.0000},
				{0.0200,3.4688,2.0000,2.4688,1.7600,-1.0000,0.0000,2.4688,1.7600,-1.0000,0.0000,2.4688,1.7600,-1.0000,0.0000,0.0000},
				{0.0200,3.5036,2.0000,2.5036,1.7400,-1.0000,-0.0000,2.5036,1.7400,-1.0000,0.0000,2.5036,1.7400,-1.0000,-0.0000,0.0000},
				{0.0200,3.5380,2.0000,2.5380,1.7200,-1.0000,0.0000,2.5380,1.7200,-1.0000,0.0000,2.5380,1.7200,-1.0000,0.0000,0.0000},
				{0.0200,3.5720,2.0000,2.5720,1.7000,-1.0000,-0.0000,2.5720,1.7000,-1.0000,0.0000,2.5720,1.7000,-1.0000,-0.0000,0.0000},
				{0.0200,3.6056,2.0000,2.6056,1.6800,-1.0000,0.0000,2.6056,1.6800,-1.0000,0.0000,2.6056,1.6800,-1.0000,0.0000,0.0000},
				{0.0200,3.6388,2.0000,2.6388,1.6600,-1.0000,-0.0000,2.6388,1.6600,-1.0000,0.0000,2.6388,1.6600,-1.0000,-0.0000,0.0000},
				{0.0200,3.6716,2.0000,2.6716,1.6400,-1.0000,0.0000,2.6716,1.6400,-1.0000,0.0000,2.6716,1.6400,-1.0000,0.0000,0.0000},
				{0.0200,3.7040,2.0000,2.7040,1.6200,-1.0000,0.0000,2.7040,1.6200,-1.0000,0.0000,2.7040,1.6200,-1.0000,0.0000,0.0000},
				{0.0200,3.7360,2.0000,2.7360,1.6000,-1.0000,0.0000,2.7360,1.6000,-1.0000,0.0000,2.7360,1.6000,-1.0000,0.0000,0.0000},
				{0.0200,3.7676,2.0000,2.7676,1.5800,-1.0000,0.0000,2.7676,1.5800,-1.0000,0.0000,2.7676,1.5800,-1.0000,0.0000,0.0000},
				{0.0200,3.7988,2.0000,2.7988,1.5600,-1.0000,0.0000,2.7988,1.5600,-1.0000,0.0000,2.7988,1.5600,-1.0000,0.0000,0.0000},
				{0.0200,3.8296,2.0000,2.8296,1.5400,-1.0000,-0.0000,2.8296,1.5400,-1.0000,0.0000,2.8296,1.5400,-1.0000,-0.0000,0.0000},
				{0.0200,3.8600,2.0000,2.8600,1.5200,-1.0000,0.0000,2.8600,1.5200,-1.0000,0.0000,2.8600,1.5200,-1.0000,0.0000,0.0000},
				{0.0200,3.8900,2.0000,2.8900,1.5000,-1.0000,-0.0000,2.8900,1.5000,-1.0000,0.0000,2.8900,1.5000,-1.0000,-0.0000,0.0000},
				{0.0200,3.9196,2.0000,2.9196,1.4800,-1.0000,0.0000,2.9196,1.4800,-1.0000,0.0000,2.9196,1.4800,-1.0000,0.0000,0.0000},
				{0.0200,3.9488,2.0000,2.9488,1.4600,-1.0000,0.0000,2.9488,1.4600,-1.0000,0.0000,2.9488,1.4600,-1.0000,0.0000,0.0000},
				{0.0200,3.9776,2.0000,2.9776,1.4400,-1.0000,-0.0000,2.9776,1.4400,-1.0000,0.0000,2.9776,1.4400,-1.0000,-0.0000,0.0000},
				{0.0200,4.0060,2.0000,3.0060,1.4200,-1.0000,0.0000,3.0060,1.4200,-1.0000,0.0000,3.0060,1.4200,-1.0000,0.0000,0.0000},
				{0.0200,4.0340,2.0000,3.0340,1.4000,-1.0000,0.0000,3.0340,1.4000,-1.0000,0.0000,3.0340,1.4000,-1.0000,0.0000,0.0000},
				{0.0200,4.0616,2.0000,3.0616,1.3800,-1.0000,-0.0000,3.0616,1.3800,-1.0000,0.0000,3.0616,1.3800,-1.0000,-0.0000,0.0000},
				{0.0200,4.0888,2.0000,3.0888,1.3600,-1.0000,0.0000,3.0888,1.3600,-1.0000,0.0000,3.0888,1.3600,-1.0000,0.0000,0.0000},
				{0.0200,4.1156,2.0000,3.1156,1.3400,-1.0000,0.0000,3.1156,1.3400,-1.0000,0.0000,3.1156,1.3400,-1.0000,0.0000,0.0000},
				{0.0200,4.1420,2.0000,3.1420,1.3200,-1.0000,-0.0000,3.1420,1.3200,-1.0000,0.0000,3.1420,1.3200,-1.0000,-0.0000,0.0000},
				{0.0200,4.1680,2.0000,3.1680,1.3000,-1.0000,-0.0000,3.1680,1.3000,-1.0000,0.0000,3.1680,1.3000,-1.0000,-0.0000,0.0000},
				{0.0200,4.1936,2.0000,3.1936,1.2800,-1.0000,0.0000,3.1936,1.2800,-1.0000,0.0000,3.1936,1.2800,-1.0000,0.0000,0.0000},
				{0.0200,4.2188,2.0000,3.2188,1.2600,-1.0000,0.0000,3.2188,1.2600,-1.0000,0.0000,3.2188,1.2600,-1.0000,0.0000,0.0000},
				{0.0200,4.2436,2.0000,3.2436,1.2400,-1.0000,-0.0000,3.2436,1.2400,-1.0000,0.0000,3.2436,1.2400,-1.0000,-0.0000,0.0000},
				{0.0200,4.2680,2.0000,3.2680,1.2200,-1.0000,0.0000,3.2680,1.2200,-1.0000,0.0000,3.2680,1.2200,-1.0000,0.0000,0.0000},
				{0.0200,4.2920,2.0000,3.2920,1.2000,-1.0000,0.0000,3.2920,1.2000,-1.0000,0.0000,3.2920,1.2000,-1.0000,0.0000,0.0000},
				{0.0200,4.3156,2.0000,3.3156,1.1800,-1.0000,0.0000,3.3156,1.1800,-1.0000,0.0000,3.3156,1.1800,-1.0000,0.0000,0.0000},
				{0.0200,4.3388,2.0000,3.3388,1.1600,-1.0000,-0.0000,3.3388,1.1600,-1.0000,0.0000,3.3388,1.1600,-1.0000,-0.0000,0.0000},
				{0.0200,4.3616,2.0000,3.3616,1.1400,-1.0000,0.0000,3.3616,1.1400,-1.0000,0.0000,3.3616,1.1400,-1.0000,0.0000,0.0000},
				{0.0200,4.3840,2.0000,3.3840,1.1200,-1.0000,-0.0000,3.3840,1.1200,-1.0000,0.0000,3.3840,1.1200,-1.0000,-0.0000,0.0000},
				{0.0200,4.4060,2.0000,3.4060,1.1000,-1.0000,-0.0000,3.4060,1.1000,-1.0000,0.0000,3.4060,1.1000,-1.0000,-0.0000,0.0000},
				{0.0200,4.4276,2.0000,3.4276,1.0800,-1.0000,0.0000,3.4276,1.0800,-1.0000,0.0000,3.4276,1.0800,-1.0000,0.0000,0.0000},
				{0.0200,4.4488,2.0000,3.4488,1.0600,-1.0000,0.0000,3.4488,1.0600,-1.0000,0.0000,3.4488,1.0600,-1.0000,0.0000,0.0000},
				{0.0200,4.4696,2.0000,3.4696,1.0400,-1.0000,-0.0000,3.4696,1.0400,-1.0000,0.0000,3.4696,1.0400,-1.0000,-0.0000,0.0000},
				{0.0200,4.4900,2.0000,3.4900,1.0200,-1.0000,0.0000,3.4900,1.0200,-1.0000,0.0000,3.4900,1.0200,-1.0000,0.0000,0.0000},
				{0.0200,4.5100,2.0000,3.5100,1.0000,-1.0000,0.0000,3.5100,1.0000,-1.0000,0.0000,3.5100,1.0000,-1.0000,0.0000,0.0000},
				{0.0200,4.5296,2.0000,3.5296,0.9800,-1.0000,-0.0000,3.5296,0.9800,-1.0000,0.0000,3.5296,0.9800,-1.0000,-0.0000,0.0000},
				{0.0200,4.5488,2.0000,3.5488,0.9600,-1.0000,0.0000,3.5488,0.9600,-1.0000,0.0000,3.5488,0.9600,-1.0000,0.0000,0.0000},
				{0.0200,4.5676,2.0000,3.5676,0.9400,-1.0000,0.0000,3.5676,0.9400,-1.0000,0.0000,3.5676,0.9400,-1.0000,0.0000,0.0000},
				{0.0200,4.5860,2.0000,3.5860,0.9200,-1.0000,-0.0000,3.5860,0.9200,-1.0000,0.0000,3.5860,0.9200,-1.0000,-0.0000,0.0000},
				{0.0200,4.6040,2.0000,3.6040,0.9000,-1.0000,-0.0000,3.6040,0.9000,-1.0000,0.0000,3.6040,0.9000,-1.0000,-0.0000,0.0000},
				{0.0200,4.6216,2.0000,3.6216,0.8800,-1.0000,0.0000,3.6216,0.8800,-1.0000,0.0000,3.6216,0.8800,-1.0000,0.0000,0.0000},
				{0.0200,4.6388,2.0000,3.6388,0.8600,-1.0000,0.0000,3.6388,0.8600,-1.0000,0.0000,3.6388,0.8600,-1.0000,0.0000,0.0000},
				{0.0200,4.6556,2.0000,3.6556,0.8400,-1.0000,-0.0000,3.6556,0.8400,-1.0000,0.0000,3.6556,0.8400,-1.0000,-0.0000,0.0000},
				{0.0200,4.6720,2.0000,3.6720,0.8200,-1.0000,0.0000,3.6720,0.8200,-1.0000,0.0000,3.6720,0.8200,-1.0000,0.0000,0.0000},
				{0.0200,4.6880,2.0000,3.6880,0.8000,-1.0000,0.0000,3.6880,0.8000,-1.0000,0.0000,3.6880,0.8000,-1.0000,0.0000,0.0000},
				{0.0200,4.7036,2.0000,3.7036,0.7800,-1.0000,0.0000,3.7036,0.7800,-1.0000,0.0000,3.7036,0.7800,-1.0000,0.0000,0.0000},
				{0.0200,4.7188,2.0000,3.7188,0.7600,-1.0000,-0.0000,3.7188,0.7600,-1.0000,0.0000,3.7188,0.7600,-1.0000,-0.0000,0.0000},
				{0.0200,4.7336,2.0000,3.7336,0.7400,-1.0000,0.0000,3.7336,0.7400,-1.0000,0.0000,3.7336,0.7400,-1.0000,0.0000,0.0000},
				{0.0200,4.7480,2.0000,3.7480,0.7200,-1.0000,0.0000,3.7480,0.7200,-1.0000,0.0000,3.7480,0.7200,-1.0000,0.0000,0.0000},
				{0.0200,4.7620,2.0000,3.7620,0.7000,-1.0000,-0.0000,3.7620,0.7000,-1.0000,0.0000,3.7620,0.7000,-1.0000,-0.0000,0.0000},
				{0.0200,4.7756,2.0000,3.7756,0.6800,-1.0000,0.0000,3.7756,0.6800,-1.0000,0.0000,3.7756,0.6800,-1.0000,0.0000,0.0000},
				{0.0200,4.7888,2.0000,3.7888,0.6600,-1.0000,-0.0000,3.7888,0.6600,-1.0000,0.0000,3.7888,0.6600,-1.0000,-0.0000,0.0000},
				{0.0200,4.8016,2.0000,3.8016,0.6400,-1.0000,0.0000,3.8016,0.6400,-1.0000,0.0000,3.8016,0.6400,-1.0000,0.0000,0.0000},
				{0.0200,4.8140,2.0000,3.8140,0.6200,-1.0000,0.0000,3.8140,0.6200,-1.0000,0.0000,3.8140,0.6200,-1.0000,0.0000,0.0000},
				{0.0200,4.8260,2.0000,3.8260,0.6000,-1.0000,0.0000,3.8260,0.6000,-1.0000,0.0000,3.8260,0.6000,-1.0000,0.0000,0.0000},
				{0.0200,4.8376,2.0000,3.8376,0.5800,-1.0000,-0.0000,3.8376,0.5800,-1.0000,0.0000,3.8376,0.5800,-1.0000,-0.0000,0.0000},
				{0.0200,4.8488,2.0000,3.8488,0.5600,-1.0000,0.0000,3.8488,0.5600,-1.0000,0.0000,3.8488,0.5600,-1.0000,0.0000,0.0000},
				{0.0200,4.8596,2.0000,3.8596,0.5400,-1.0000,0.0000,3.8596,0.5400,-1.0000,0.0000,3.8596,0.5400,-1.0000,0.0000,0.0000},
				{0.0200,4.8700,2.0000,3.8700,0.5200,-1.0000,-0.0000,3.8700,0.5200,-1.0000,0.0000,3.8700,0.5200,-1.0000,-0.0000,0.0000},
				{0.0200,4.8800,2.0000,3.8800,0.5000,-1.0000,-0.0000,3.8800,0.5000,-1.0000,0.0000,3.8800,0.5000,-1.0000,-0.0000,0.0000},
				{0.0200,4.8896,2.0000,3.8896,0.4800,-1.0000,0.0000,3.8896,0.4800,-1.0000,0.0000,3.8896,0.4800,-1.0000,0.0000,0.0000},
				{0.0200,4.8988,2.0000,3.8988,0.4600,-1.0000,0.0000,3.8988,0.4600,-1.0000,0.0000,3.8988,0.4600,-1.0000,0.0000,0.0000},
				{0.0200,4.9076,2.0000,3.9076,0.4400,-1.0000,-0.0000,3.9076,0.4400,-1.0000,0.0000,3.9076,0.4400,-1.0000,-0.0000,0.0000},
				{0.0200,4.9160,2.0000,3.9160,0.4200,-1.0000,0.0000,3.9160,0.4200,-1.0000,0.0000,3.9160,0.4200,-1.0000,0.0000,0.0000},
				{0.0200,4.9240,2.0000,3.9240,0.4000,-1.0000,0.0000,3.9240,0.4000,-1.0000,0.0000,3.9240,0.4000,-1.0000,0.0000,0.0000},
				{0.0200,4.9316,2.0000,3.9316,0.3800,-1.0000,0.0000,3.9316,0.3800,-1.0000,0.0000,3.9316,0.3800,-1.0000,0.0000,0.0000},
				{0.0200,4.9388,2.0000,3.9388,0.3600,-1.0000,0.0000,3.9388,0.3600,-1.0000,0.0000,3.9388,0.3600,-1.0000,0.0000,0.0000},
				{0.0200,4.9456,2.0000,3.9456,0.3400,-1.0000,-0.0000,3.9456,0.3400,-1.0000,0.0000,3.9456,0.3400,-1.0000,-0.0000,0.0000},
				{0.0200,4.9520,2.0000,3.9520,0.3200,-1.0000,0.0000,3.9520,0.3200,-1.0000,0.0000,3.9520,0.3200,-1.0000,0.0000,0.0000},
				{0.0200,4.9580,2.0000,3.9580,0.3000,-1.0000,-0.0000,3.9580,0.3000,-1.0000,0.0000,3.9580,0.3000,-1.0000,-0.0000,0.0000},
				{0.0200,4.9636,2.0000,3.9636,0.2800,-1.0000,0.0000,3.9636,0.2800,-1.0000,0.0000,3.9636,0.2800,-1.0000,0.0000,0.0000},
				{0.0200,4.9688,2.0000,3.9688,0.2600,-1.0000,-0.0000,3.9688,0.2600,-1.0000,0.0000,3.9688,0.2600,-1.0000,-0.0000,0.0000},
				{0.0200,4.9736,2.0000,3.9736,0.2400,-1.0000,0.0000,3.9736,0.2400,-1.0000,0.0000,3.9736,0.2400,-1.0000,0.0000,0.0000},
				{0.0200,4.9780,2.0000,3.9780,0.2200,-1.0000,0.0000,3.9780,0.2200,-1.0000,0.0000,3.9780,0.2200,-1.0000,0.0000,0.0000},
				{0.0200,4.9820,2.0000,3.9820,0.2000,-1.0000,0.0000,3.9820,0.2000,-1.0000,0.0000,3.9820,0.2000,-1.0000,0.0000,0.0000},
				{0.0200,4.9856,2.0000,3.9856,0.1800,-1.0000,0.0000,3.9856,0.1800,-1.0000,0.0000,3.9856,0.1800,-1.0000,0.0000,0.0000},
				{0.0200,4.9888,2.0000,3.9888,0.1600,-1.0000,0.0000,3.9888,0.1600,-1.0000,0.0000,3.9888,0.1600,-1.0000,0.0000,0.0000},
				{0.0200,4.9916,2.0000,3.9916,0.1400,-1.0000,-0.0000,3.9916,0.1400,-1.0000,0.0000,3.9916,0.1400,-1.0000,-0.0000,0.0000},
				{0.0200,4.9940,2.0000,3.9940,0.1200,-1.0000,0.0000,3.9940,0.1200,-1.0000,0.0000,3.9940,0.1200,-1.0000,0.0000,0.0000},
				{0.0200,4.9960,2.0000,3.9960,0.1000,-1.0000,-0.0000,3.9960,0.1000,-1.0000,0.0000,3.9960,0.1000,-1.0000,-0.0000,0.0000},
				{0.0200,4.9976,2.0000,3.9976,0.0800,-1.0000,0.0000,3.9976,0.0800,-1.0000,0.0000,3.9976,0.0800,-1.0000,0.0000,0.0000},
				{0.0200,4.9988,2.0000,3.9988,0.0600,-1.0000,-0.0000,3.9988,0.0600,-1.0000,0.0000,3.9988,0.0600,-1.0000,-0.0000,0.0000},
				{0.0200,4.9996,2.0000,3.9996,0.0400,-1.0000,0.0000,3.9996,0.0400,-1.0000,0.0000,3.9996,0.0400,-1.0000,0.0000,0.0000},
				{0.0200,5.0000,2.0000,4.0000,0.0200,-1.0000,-0.0000,4.0000,0.0200,-1.0000,0.0000,4.0000,0.0200,-1.0000,-0.0000,0.0000},
				{0.0200,5.0000,2.0000,4.0000,0.0000,-1.0000,0.0000,4.0000,-0.0000,-1.0000,0.0000,4.0000,0.0000,-1.0000,0.0000,0.0000},

	    };

	@Override
	public double[][] getPath() {
	    return points;
	}
}
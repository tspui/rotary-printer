#!/usr/bin/perl -i~


use strict;
use warnings;
use autodie;
use Data::Dumper;
use Clone qw(clone);
use CAD::Calc (
					qw(
						dist2d
						line_vec
						distdivide
						cart_to_pol
						)
					);





					

my $filename = 'owl.gcode';#keychain2 is benchmark
open my $fh, '<', $filename;

					
my %pos = (X => 0, Y => 0, Z => 0, E => 0, F => 0);
my %lastpos;

my $y_conversion = 120;
my $radius_conversion = 1;
my $z_factor = 1;#1
my $e_factor = 1; #0.95
my $f_factor = 1;#3
my $F_convert = 1; #23;
my $F_limit = 1600; #3000
my $new_speed = 1600;
my $pi = 3.14159;
my $E_retract = 0.2;  #0.95
my $Z_Max = 135.1; # 151.5,  109 for new printer
my $X_Max = 101.0; #120, 107 for new printer

my $mindist = 1;
my $mindistsq = $mindist * $mindist;
my $ptP = [1,1];
my $ptQ = [2,1];
my $ptPE;
my $ptQE;
my $ptPF;
my $ptPZ;
my $e = 0;

my $las_theta;
my $las_ptheta;
my $cycle = 2;
my $y_count = 0;

# my $count_confirm = 0;

my $offset = 25;

open STDOUT, ">owlout.gcode";

				
				


while (<$fh>)
	{

		
		if (m#\bG[01]\b#)
			{						

				while (m#([XYZEF])(-?\d+(\.\d+)?)#gi)
					{	# take in all XYZEF as keys, and the corresponding numbers is values and store it in %pos.
						$pos{uc $1} = $2;
					}
						
				if (!keys %lastpos)
				{	#if %lastpos is empty, set it = %pos.

						%lastpos = %{ clone (\%pos) };		
				}

		
			
			 
			 $ptP = [$pos{X} +$offset,  $pos{Y}+$offset];
#printf "ptx=$pos{X}\n\n";
#printf"pty=$pos{Y}\n\n";
			 $ptQ = [$lastpos{X} +$offset,  $lastpos{Y}+$offset];
			 $ptPE = $pos{E}*$e_factor;
			 $ptQE = $lastpos{E}*$e_factor;
			 $ptPF = $pos{F}*$f_factor;
			 $ptPZ = $Z_Max - $pos{Z}*$z_factor;
			
			#if($ptPE == $ptQE)
			#{
				#$ptPE = $ptPE - $E_retract;
				#$ptQE = $ptPE;
				#printf "G1 F%.3f X%.3f\n", $F_limit, $X_Max; 
				#printf "G1 F%.3f E%.5f ;retraction triggered\n", $F_limit, $ptPE; 
			#}
			
			if (/X/ && /Y/ != 0) {
				
			my $length = dist2d($ptP, $ptQ);

				#$line is of the form:  [ [x1, y1, z1], [x2, y2, z2] ] where z1 and z2 are option
				my $count = $length / $mindist;
				$count = int($count);
				 #printf "Count = $count\n\n";
				
				if ($count != 0) 	# DONE> if segment to divide is zero, skip newpoints creation.
				{

				my $newline = [$ptQ, $ptP];
				my @newpoints = distdivide($newline, $mindist);
				

				my $ptQE_incremented = $ptQE;
				
				foreach (@newpoints) {
					# $count_confirm ++;
					# print "CC $count_confirm ";
					my $x = @$_[0];
					my $y = @$_[1];
									
						
					my ($radius, $theta) = cart_to_pol($x, $y);
					
					# eliminate unnecessary back rotation
					
					if  (!$las_theta)
					{
						$las_theta = $theta;
					}
					
					if ($theta - $las_theta >= $cycle)
					{
						$y_count--;
					}
					
					elsif ($theta - $las_theta <= -$cycle)
					{
						$y_count++;
					}
					else{}	
					my $theta_convert = ($theta + 2*$pi*$y_count)/$pi*$y_conversion; # conversion factor found empirically
					
					# e treatment
							
					if($ptPE == $ptQE)
					{
						#printf "G1 F%.3f E%.5f ;retraction triggered\n", $F_limit, $ptPE; 
							 #printf "G1 X%.3f  Y%.3f\n", $radius, $theta_convert;
					}
					else
					{
						$e = $ptQE_incremented + ($ptPE - $ptQE)/$count;
						

						if ($ptPF>$F_limit) {
							$ptPF= $F_limit;
						}
						else {}
							
						printf "G1 X%.3f  Y%.3f F%.3f E%.5f\n", ($X_Max - $radius)*$radius_conversion, $theta_convert, $ptPF, $e;#$new_speed, $e; 
						$ptQE_incremented = $e;
					}
					

					

					
					$las_theta = $theta;
				}
				# $count_confirm = 0;
				}
				
				my $ptPx = @$ptP[0];
				my $ptPy = @$ptP[1];
					my ($pradius, $ptheta) = cart_to_pol($ptPx, $ptPy);
					
					if  (!$las_theta)
					{
						$las_theta = $ptheta;
					}
					
					if ($ptheta - $las_theta >= $cycle)
					{
						$y_count--;
					}
					
					elsif ($ptheta - $las_theta <= -$cycle)
					{
						$y_count++;
					}
					else{}	
					
					my $ptheta_convert = ($ptheta + 2*$pi*$y_count)/$pi*$y_conversion;
				

				
	my $pFf =  $ptPF;

						if ($pFf>$F_limit) {
							$pFf= $F_limit;
						}
						else {}
						
				printf "G1 Z%.3f X%.3f  Y%.3f F%.3f E%.5f\n", $ptPZ, ($X_Max - $pradius)*$radius_conversion, $ptheta_convert, $pFf, $ptPE; #$new_speed, $ptPE;   
				# printf "G1 Z%.3f X%.3f  Y%.3f F%.3f E%.5f\n", $ptPZ, $X_Max - $pradius, $ptheta, $ptPF, $ptPE; 
				
				$las_theta = $ptheta;
				
				%lastpos = %{ clone (\%pos) };	

			
				
				
			}
			else {
				printf "G1 Z%.3f F%.3f\n", $ptPZ, $F_limit;#$ptPF;
				#printf "G1 Z%.3f F%.3f E%.5f\n", $ptPZ, $ptPF, $ptPE;
			}
		}
		elsif (m#\bM109\b#)
		{}
		
		else {
			print;
		}
		
		
		
	}
printf "G0 X0 Z0\n";
printf "M104 S25\n";
	close $fh or die "Could not close '$filename': S!";

close STDOUT;

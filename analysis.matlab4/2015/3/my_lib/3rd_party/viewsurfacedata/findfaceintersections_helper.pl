#!/usr/bin/perl -w

# perl findfaceintersections_helper.pl file.dat
#
# file.dat is a binary file in double format.
# we overwrite file.dat with the answer.
#
# see findfaceintersections.m for more details.
#
# this function is UGLY.  FIXME.
# IDEA: use java in matlab as hash??

use strict;

# internal constants
my $readchunk = 100000;

# deal with input
die if (scalar(@ARGV)!=1);
my $file = $ARGV[0];

# read file
open(FILE,"<$file");
binmode(FILE); 
my $inputraw;
my $inputtemp;
while (read(FILE,$inputtemp,$readchunk)) {
  $inputraw = $inputraw.$inputtemp;
}
close(FILE);
my @input = unpack("d*",$inputraw);

# define connections (hashes)
my $interps;
my $connections;
for (my $p=0;$p<scalar(@input);$p=$p+6) {
  my @ia = sort(@input[$p+1..$p+2]);
  my @ja = sort(@input[$p+4..$p+5]);
  my $i = join('/',@ia);
  my $j = join('/',@ja);
  if ($input[$p+1] == $ia[0]) {
    $interps->{$i} = $input[$p];  # this in effect requires that each face is intersected at most once
  } else {
    $interps->{$i} = 1-$input[$p];
  }
  if ($input[$p+4] == $ja[0]) {
    $interps->{$j} = $input[$p+3];
  } else {
    $interps->{$j} = 1-$input[$p+3];
  }
  if (not defined $connections->{$i}) { $connections->{$i} = []; }
  if (not defined $connections->{$j}) { $connections->{$j} = []; }
  push(@{$connections->{$i}},$j);
  push(@{$connections->{$j}},$i);
}

# sanity check
foreach (keys(%{$connections})) {
  my $temp = scalar(@{$connections->{$_}});
  if (not ($temp==1 or $temp==2)) {
    die("weird connections case found!");
  }
}

# define output
my $output;

# search for components
my $keys;
while (scalar(@{$keys = [keys(%{$connections})]}) != 0) {
  my $v1 = $keys->[0];                  # first vertex
  my $polygon = [];                     # initialize polygon
  
  # try first neighbor (which must exist)
  $polygon = searchhelper($v1,$connections->{$v1}->[0],$connections);
  # construct full list
  unshift(@{$polygon},$v1);
  # if not circular case
  if ($v1 ne $polygon->[scalar(@{$polygon})-1]) {
    # if possible, do second neighbor, reverse, and tack on to beginning
    if (scalar(@{$connections->{$v1}}) > 1) {
      unshift(@{$polygon},reverse(@{searchhelper($v1,$connections->{$v1}->[1],$connections)}));
    }
  }
  
  # clean up
  for (my $p=0;$p<scalar(@{$polygon});$p++) {
    delete $connections->{$polygon->[$p]};
  }
  
  # output
  $output .= pack("d*",scalar(@{$polygon}));
  for (my $p=0;$p<scalar(@{$polygon});$p++) {
    $output .= pack("d*",$interps->{$polygon->[$p]});
    $output .= pack("d*",split(/\//,$polygon->[$p]));
  }
}

# write output
open(FILE,">$file");
binmode(FILE);
syswrite(FILE,$output);
close(FILE);

########################

# first is the vertex to start from
# second is a neighbor of first
# connections is the hash
#
# return ref to array of vertices like [ second third ... ]
# where the last vertex could be be identical to the first vertex.
sub searchhelper {
  my ($first,$second,$connections) = @_;

  my $prev = $first;
  my $cur = $second;
  my $polygon = [];
  while (1) {
    # add to polygon
    push(@{$polygon},$cur);
    # check for circularity
    if ($cur eq $first) {
#      print("circularity found\n");
      last;
    }
    # try to continue
    my $next;
    foreach (@{$connections->{$cur}}) {
      if ($_ ne $prev) {
        $next = $_;                      # next one
        last;
      }
    }
    # if we didn't find a new guy
    if (not defined $next) {
      last;
    }
    # if we found a new guy
    $prev = $cur;
    $cur = $next;
  }
#  print("done: ".scalar(@{$polygon})."\n");
  
  return $polygon;
}

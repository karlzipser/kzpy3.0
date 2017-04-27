#!/usr/bin/perl -w

# perl getbyteorder.pl
#
# return the native machine format,
#   either "l" or "b"

use strict;
use Config;

my $byteorder = $Config{byteorder};
if ($byteorder eq "1234" or $byteorder eq "12345678") {
  print "l";
} elsif ($byteorder eq "4321" or $byteorder eq "87654321") {
  print "b";
}

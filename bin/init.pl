use FindBin qw($Bin);
use lib "$Bin/../lib";

use strict;
use warnings;
use 5.010;
use POSIX;

use AutoPilot::Frames::QuadRotorHFrame;

my $pilot = AutoPilot::Frames::QuadRotorHFrame->new(
);
use Data::Dumper;
say Dumper $pilot->propulsionDrives();

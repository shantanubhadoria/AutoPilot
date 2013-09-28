use FindBin qw($Bin);
use lib "$Bin/../lib";

use strict;
use warnings;

use AutoPilot;

my $pilot = AutoPilot->new(
    magnetometer => 'LSM303DLHC', 
);

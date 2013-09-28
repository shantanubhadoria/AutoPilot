package AutoPilot;

# PODNAME: AutoPilot
# ABSTRACT: Generic perl autopilot framework
# COPYRIGHT
# VERSION

use 5.010;
use Moose;

# Dependencies
use Module::Pluggable::Object;

has magnetometer => (
    is => 'ro',
);

has altimeter => (
    is => 'ro',
);

1;

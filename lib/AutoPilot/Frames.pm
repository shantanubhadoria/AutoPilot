use strict;
use warnings;
package AutoPilot::Frames;

# PODNAME: AutoPilot::Frames
# ABSTRACT: Generic Object for a frame hosting the autopilot system
# COPYRIGHT
# VERSION

# Dependencies
use 5.010;
use Moose;

=attr propulsionDrives

List(ArrayRef) of Propulsion Drives attached to the Frame see
L<AutoPilot::Propulsion>

=cut

has propulsionDrives => (
    is         => 'rw',
    traits     => ['Array'],
    lazy_build => 1,
); 

# Placeholder
sub _build_propulsionDrives {
}

1;

use strict;
use warnings;
package AutoPilot::Propulsion::Propeller;

# PODNAME: AutoPilot::Propulsion::Propeller
# ABSTRACT: Object for a individual Propellor based propulsion drive on a body
# COPYRIGHT
# VERSION

# Dependencies
use 5.010;
use Moose;

use Math::Trig qw/deg2rad/;

extends 'AutoPilot::Propulsion';

=attr angleAgainstX

angle in degrees on XY plane that the propeller makes against the roll axis

=cut

has angleAgainstX => (
    is      => 'rw',
    trigger => \&_trigger_angleAgainstX,
);

sub _trigger_angleAgainstX {
    my ( $self, $angleAgainstX ) = @_;

    $self->rollFactor( cos( deg2rad( $angleAgainstX + 90 ) ) );
    $self->pitchFactor( cos( deg2rad( $angleAgainstX ) ) );
}

1;

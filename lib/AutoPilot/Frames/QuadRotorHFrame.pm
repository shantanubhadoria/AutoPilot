use strict;
use warnings;
package AutoPilot::Frames::QuadRotorHFrame;

# PODNAME: AutoPilot::Frames:QuadRotorHFrame
# ABSTRACT: Object for a quad rotor in H-Frame configuration
# COPYRIGHT
# VERSION

# Dependencies
use 5.010;
use Moose;
use AutoPilot::Propulsion::Propeller;

extends 'AutoPilot::Frames';

=method _build_propulsionDrives

This local method overrides  the builder for propulsion drives in base class
L<AutoPilot::Frames>

=cut

sub _build_propulsionDrives{
    my ( $self ) = @_;
    return [
            AutoPilot::Propulsion::Propeller->new(
                angleAgainstX => 45, # front right
                yawFactor     => 1, # clockwise
            ),
            AutoPilot::Propulsion::Propeller->new(
                angleAgainstX => -45, # front left
                yawFactor     => -1, # counter clockwise
            ),
            AutoPilot::Propulsion::Propeller->new(
                angleAgainstX => 135, # rear right 
                yawFactor     => -1, # counter clockwise
            ),
            AutoPilot::Propulsion::Propeller->new(
                angleAgainstX => -135, # rear left
                yawFactor     => 1, # clockwise
            ),
        ];
}

1;

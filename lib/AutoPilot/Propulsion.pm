use strict;
use warnings;
package AutoPilot::Propulsion;

# PODNAME: AutoPilot::Propulsion
# ABSTRACT: Object for a individual propulsion drive on a body
# COPYRIGHT
# VERSION

# Dependencies
use 5.010;
use Moose;

=attr enabled

Is the motor enabled at the moment?

=cut

has enabled => (
);

=attr rollFactor

This is the roll torque applied by the drive on the body as a multiple of its
speed. For a drive. This can be a fraction or a whole number. This can be set to
any number as long as in a stable body when we take a aggregate of speeds
multiplied by roll factor for all drives the net roll is zero.

=cut

has rollFactor => (
);

=attr pitchFactor

This is the pitch torque applied by the drive on the body as a multiple of its
speed. For a drive. This can be a fraction or a whole number. This can be set to
any number as long as in a stable body when we take a aggregate of speeds
multiplied by pitch factor for all drives the net roll is zero.

=cut

has pitchFactor => (
);

=attr yawFactor

This is the yaw torque applied by the drive on the body as a multiple of its
speed. For a drive. This can be a fraction or a whole number. This can be set to
any number as long as in a stable body when we take a aggregate of speeds
multiplied by yaw factor for all drives the net roll is zero.

=cut

has yawFactor => (
);


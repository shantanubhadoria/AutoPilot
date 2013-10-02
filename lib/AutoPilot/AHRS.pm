use strict;
use warnings;
package AutoPilot::AHRS;

# PODNAME: AutoPilot::AHRS
# ABSTRACT: Autopilot Altitude and Heading Reference(AHRS) framework
# COPYRIGHT
# VERSION

# Dependencies
use 5.010;
use Moose;

use Math::Vector::Real;
use Math::Vector::Real::XS; # we do not need to use this explicitly but its mentioned to allow dzil to compile dependency
use Math::Matrix;

has gyroscope => (
    is         => 'ro',
);

has gyroscopeVector => (
    is         => 'rw',
);

has accelerometer => (
    is         => 'ro',
);

has accelerometerVector => (
    is         => 'rw',
);

has magnetometer => (
    is         => 'ro',
);

has magnetometerVector => (
    is         => 'rw',
);

has omegaVector => (
    is         => 'rw',
    default    => sub {
        return [0,0,0];
    },
);

has omegaP => (
    is         => 'rw',
    default    => sub {
        return [0,0,0];
    },
);

has omega => (
    is         => 'rw',
    default    => sub {
        return [0,0,0];
    },
);

has omegaI => (
    is         => 'rw',
    default    => sub {
        return [0,0,0];
    },
);

has updateMatrix => (
    is         => 'rw',
    default    => sub {
        return [ 
            [0,1,2],
            [3,4,5],
            [6,7,8],
        ];
    },
);

sub heading {
    my ( $self ) = @_;
    my $gyroRadiansPerSecond = $self->gyroscope->getReadingsRadiansPerSecond;
    $self->gyroscopeVector(
       [ 
            $gyroRadiansPerSecond->{x}, # roll
            $gyroRadiansPerSecond->{y}, # pitch
            $gyroRadiansPerSecond->{z}, # yaw
       ]
    );
    $self->omega( $self->gyroscopeVector + $self->omegaI ); # adding proportional term
    $self->omegaVector( $self->omega + $self->omegaP ); # adding integrator term


    $self->updateMatrix($self->updateMatrix);

}

sub matrixUpdate {
    my ( $self ) = @_;
}
sub _addMatrix {
    my ( $self ) = @_;
}

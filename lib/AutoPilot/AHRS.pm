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

use constant {
    Kp_ROLLPITCH => 0.02,
    Ki_ROLLPITCH => 0.00002,
    Kp_YAW       => 1.2,
    Ki_YAW       => 0.00002,
};
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

has deltaTime => (
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

has DCMMatrix => (
    is         => 'rw',
    default    => sub {
        return [ 
            [1,0,0],
            [0,1,0],
            [0,0,1],
        ];
    },
);

has magneticHeading => (
    is      => 'rw',
    default => 0,
);

has roll => (
    is      => 'rw',
    default => 0,
);

has pitch => (
    is      => 'rw',
    default => 0,
);

has yaw => (
    is      => 'rw',
    default => 0,
);

sub updateDCMMatrix {
    my ( $self ) = @_;
    my $deltaTime = $self->deltaTime;

    $self->omega(       _add_vector( $self->gyroscopeVector, $self->omegaI ) ); # adding proportional term
    $self->omegaVector( _add_vector( $self->omega          , $self->omegaP ) ); # adding integrator term

    my $updateMatrix = $self->updateMatrix;
    my $omegaVector  = $self->omegaVector;
    $updateMatrix->[0]->[0] = 0; 
    $updateMatrix->[0]->[1] = $deltaTime * $omegaVector->[2] * ( -1 ); # -z
    $updateMatrix->[0]->[2] = $deltaTime * $omegaVector->[1];          #  y
    $updateMatrix->[1]->[0] = $deltaTime * $omegaVector->[2];          #  z
    $updateMatrix->[1]->[1] = 0; 
    $updateMatrix->[1]->[2] = $deltaTime * $omegaVector->[0] * ( -1 ); # -x
    $updateMatrix->[2]->[0] = $deltaTime * $omegaVector->[1] * ( -1 ); # -y
    $updateMatrix->[2]->[1] = $deltaTime * $omegaVector->[0];          #  x
    $updateMatrix->[2]->[2] = 0; 

    $self->updateMatrix($updateMatrix);

    my $DCMMatrix  = $self->DCMMatrix;
    my $DCMMatrix = _addMatrix($DCMMatrix,_multiplyMatrix($DCMMatrix,$updateMatrix));
    $self->DCMMatrix($DCMMatrix);
}

sub normalize {
    my ( $self ) = @_;

    my $DCMMatrix    = $self->DCMMatrix;
    my $tempMatrix   = [[]];
    my $error        = _dotProductVector($DCMMatrix->[0],$DCMMatrix->[1]) * ( -0.5 );

    $tempMatrix->[0] = _addVector(
        _scalarProductVector($DCMMatrix->[0], $error)
        , $DCMMatrix->[0]
    );
    $tempMatrix->[1] = _addVector(
        _scalarProductVector($DCMMatrix->[1], $error)
        , $DCMMatrix->[1]
    );
    $tempMatrix->[2] = _crossProductVector($tempMatrix->[0], $tempMatrix->[1]);

    my $renorm;
    $renorm = (0.5) * (3 - _dotProductVector($tempMatrix->[0], $tempMatrix->[0]));
    $DCMMatrix->[0] = _scalarProductVector($tempMatrix->[0], $renorm);

    $renorm = (0.5) * (3 - _dotProductVector($tempMatrix->[1], $tempMatrix->[1]));
    $DCMMatrix->[1] = _scalarProductVector($tempMatrix->[1], $renorm);

    $renorm = (0.5) * (3 - _dotProductVector($tempMatrix->[2], $tempMatrix->[2]));
    $DCMMatrix->[2] = _scalarProductVector($tempMatrix->[2], $renorm);

    $self->DCMMatrix($DCMMatrix);
}

sub driftCorrection {
    my ( $self ) = @_;

    my $accelerometerVector = $self->accelerometerVector;
    my $DCMMatrix = $self->DCMMatrix;
    my $omegaP    = $self->omegaP;
    my $omegaI    = $self->omegaI;
    my $magneticHeading = $self->magneticHeading;

    my $accelerometerMagnitudeInG = sqrt(
        (($accelerometerVector->[0]) ** 2)
        + (($accelerometerVector->[0]) ** 2)
        + (($accelerometerVector->[0]) ** 2)
    );
    # Dynamic weighting of accelerometer info (reliability filter)
    # Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)

    my $accelerometerWeightage = ( $accelerometerMagnitude > 0.5 && $accelerometer <1.5 )
        ? 1 : 0;
    
    my $errorRollPitch = _crossProductVector($accelerometerVector, $DCMMatrix);
    $omegaP = _scalarProductVector(
        $errorRollPitch,
        Kp_ROLLPITCH * $accelerometerWeightage
    );
    my $scaledOmegaI = _scalarProductVector(
        $errorRollPitch,
        Ki_ROLLPITCH * $accelerometerWeightage
    );
    $omegaI = _addVector($omegaI, $scaledOmegaI);


    # *****************************YAW********************************
    # We make the gyro YAW drift correction based on compass magnetic heading
    my $magneticHeadingX = cos($mageneticHeading);
    my $magneticHeadingY = sin($mageneticHeading);

    # Applies the yaw correction on xyz position of frame depending on position.
    my $errorCourse      = 
        ($DCMMatrix->[0]->[0] * $magneticHeadingY)
        - ($DCMMatirx->[1]->[0] * $magneticHeadingX); # Calculating Yaw Error
    my $errorYaw         = _scalarProductVector($DCMMatrix->[2], $errorCourse);

    # Proportional of Yaw 
    my $scaledOmegaP     = _scalarProductVector($errorYaw, Kp_YAW); 
    $omegaP              = _addVector($omegaP, $scaledOmegaP);

    # Integrator
    $scaledOmegaI        = _scalarProductVector($errorYaw, Ki_YAW);
    $omegaI              = _addVector($omegaI, $scaledOmegaI);

    # Move values to Object
    $self->omegaI($omegaI); 
    $self->omegaP($omegaP);
}

sub eulerAngles {
    my ( $self ) = @_;
    my $DCMMatrix = $self->DCMMatrix;

    $self->roll( atan2( $DCMMatrix->[2]->[1], $DCMMatrix->[2]->[2] ) );
    $self->pitch( ( -1 ) * ( _asin( $DCMMatrix->[2]->[0] ) );
    $self->yaw( atan2( $DCMMatrix->[1]->[0], $DCMMatrix->[0]->[0] ) );
}

sub compassHeading {
    my ( $self ) = @_;
    $magnetometerVector = $self->magnetometerVector;

    $cosRoll  = cos( $self->roll );
    $sinRoll  = sin( $self->roll );
    $cosPitch = cos( $self->pitch );
    $sinPitch = sin( $self->pitch );

    $magnetometerX = ($magnetometerVector->{x} * $cosPitch)
        + ($magnetometerVector->{y} * $sinRoll * $sinPitch)
        + ($magnetometerVector->{z} * $cosRoll * $sinPitch);
    $magnetometerY = ($magnetometerVector->{y} * $cosRoll)
        - ($magnetometerVector->{z} * $sinRoll);
    $self->magneticHeading( atan2( ( ( -1 ) * $magnetometerY), $magnetometerX ) );
}

sub _addVector {
    my ( $vectorA, $vectorB ) = @_;
    my @ret;
    for (0 .. 2){
        $ret[$_] = $vectorA->[$_] + $vectorB->[$_];
    }
    return \@ret;
}

sub _crossProductVector {
    my ( $vectorA, $vectorB) = @_;
    return [
        ( $vectorA->[1] * $vectorB->[2] ) - ( $vectorA->[2] * $vectorB->[1]  ),
        ( $vectorA->[2] * $vectorB->[0] ) - ( $vectorA->[0] * $vectorB->[2]  ),
        ( $vectorA->[0] * $vectorB->[1] ) - ( $vectorA->[1] * $vectorB->[0]  ),
    ];
}

sub _dotProductVector {
    my ( $vectorA, $vectorB ) = @_;
    my $reti = 0;
    for (0 .. 2){
        $ret += $vectorA->[$_] * $vectorB->[$_];
    }
    return $ret;
}

sub _scalarProductVector {
    my ( $vector, $scalar) = @_;
    my @ret;
    for (0 .. 2){
        $ret[$_] += $vector->[$_] * $scalar;
    }
    return \@ret;
}

sub _addMatrix {
    my ( $matrixA, $matrixB ) = @_;
    my $tempMatrix = [[]];
    for my $x (0 .. 2){
        for my $y (0 .. 2){
            $tempMatrix->[$x]->[$y] = $matrixA->[$x]->[$y] + $matrixB->[$x]->[$y];
        }
    }
    return $tempMatrix;
}

sub _multiplyMatrix {
    my ( $matrixA, $matrixB ) = @_;
    my $tempMatrix = [[]];
    for my $x (0 .. 2){
        for my $y (0 .. 2){
            $tempMatrix->[$x]->[$y] = 
                ( $matrixA->[$x]->[0] * $matrixB->[0]->[$y] )
                + ( $matrixA->[$x]->[1] * $matrixB->[1]->[$y] )
                + ( $matrixA->[$x]->[2] * $matrixB->[2]->[$y] );
        }
    }
    return $tempMatrix;
}

sub _asin { atan2($_[0], sqrt(1 - $_[0] * $_[0])) }

1;

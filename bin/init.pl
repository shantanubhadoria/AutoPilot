use FindBin qw($Bin);
use lib "$Bin/../lib";

use strict;
use warnings;
use 5.010;
use POSIX;

use AutoPilot::Frames::QuadRotorHFrame;
use AutoPilot::AHRS;

use Device::Gyroscope::L3GD20;
use Device::Accelerometer::LSM303DLHC;
use Device::Magnetometer::LSM303DLHC;

my $pilot = AutoPilot::Frames::QuadRotorHFrame->new(
);
my $ahrs  = AutoPilot::AHRS->new(
    gyroscope     => Device::Gyroscope::L3GD20->new(
        I2CBusDevicePath => '/dev/i2c-1',
        xZero => -26.99,
        yZero => 7.46,
        zZero => -9.07,
    ),
    accelerometer => Device::Accelerometer::LSM303DLHC->new(
        I2CBusDevicePath => '/dev/i2c-1',
    ),
    magnetometer  => Device::Magnetometer::LSM303DLHC->new(
        I2CBusDevicePath => '/dev/i2c-1',
    ),
);

use Data::Dumper;
#say Dumper $pilot->propulsionDrives();

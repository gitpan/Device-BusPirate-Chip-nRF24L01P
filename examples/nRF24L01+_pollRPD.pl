#!/usr/bin/perl

use strict;
use warnings;

use Device::BusPirate;
use Getopt::Long;
use Data::Dump 'pp';
use Time::HiRes qw( sleep );

my $PIRATE = "/dev/ttyUSB0";
my $BAUD = 115200;
my $SPEED = "1M";

GetOptions(
   'p|pirate=s' => \$PIRATE,
   'b|baud=i'   => \$BAUD,
   's|speed=s'  => \$SPEED,

   'C|channel=i' => \(my $CHANNEL = 30),
) or exit 1;

my $pirate = Device::BusPirate->new(
   serial => $PIRATE,
   baud   => $BAUD,
);

my $nrf = $pirate->mount_chip( "nRF24L01+",
   open_drain => 0,
   speed      => $SPEED,
)->get;

$nrf->power(1)->get;
print "Power on\n";

# Power-down to reconfigure
$nrf->pwr_up( 0 )->get;
$nrf->chip_enable( 0 )->get;

$nrf->change_config(
   PRIM_RX => 1,
   RF_CH => $CHANNEL,
)->get;

$nrf->clear_caches;
printf "Config:\n%s\n%s\n", pp($nrf->read_config->get), pp($nrf->read_rx_config( 0 )->get);

$nrf->pwr_up( 1 )->get;
print "PWR_UP\n";

$nrf->chip_enable( 1 )->get;
print "CE high - entered PRX mode...\n";

$SIG{INT} = $SIG{TERM} = sub { exit };

STDOUT->autoflush(1);

while(1) {
   sleep 0.05;
   my $rpd = $nrf->rpd->get;
   print $rpd ? "X" : ".";
}

END {
   if( $nrf ) {
      $nrf->chip_enable( 0 )->get;
      $nrf->pwr_up( 0 )->get;
      $nrf->power(0)->get;
      print "Power off\n";
   }
   $pirate and $pirate->stop;
}

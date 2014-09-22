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

   'A|address=s' => \my $ADDRESS,
   'C|channel=i' => \(my $CHANNEL = 30),
   'D|dpl'       => \my $DPL,
) or exit 1;

my $AW = scalar( split m/:/, $ADDRESS );
$AW >= 3 and $AW <= 6 or die "Invalid address - must be 3 to 5 octets\n";

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
   RF_DR   => "2E6",  # 2 Mbit/sec,
   RF_CH   => $CHANNEL,
   AW      => $AW,
   EN_DPL  => $DPL,
)->get;

$nrf->change_rx_config( 0,
   RX_ADDR => $ADDRESS,
   ( $DPL ?
      ( DYNPD   => 1 ) :
      ( RX_PW   => 1 ) ),
)->get;

$nrf->clear_caches;
printf "PRX config:\n%s\n%s\n", pp($nrf->read_config->get), pp($nrf->read_rx_config( 0 )->get);

printf "Listening on channel %d address %s\n",
   $nrf->read_config->get->{RF_CH}, $nrf->read_rx_config( 0 )->get->{RX_ADDR};

$nrf->flush_rx_fifo->get;

$nrf->reset_interrupt->get;

$nrf->pwr_up( 1 )->get;
print "PWR_UP\n";

$nrf->chip_enable( 1 )->get;
print "CE high - entered PRX mode...\n";

$SIG{INT} = $SIG{TERM} = sub { exit };

while(1) {
   sleep 0.05 until $nrf->read_status->get->{RX_DR};
   print "Packet received...\n";

   my $plen;
   if( $DPL ) {
      $plen = $nrf->read_rx_payload_width->get;
      print "Dynamic payload length $plen\n";
   }
   else {
      $plen = 1;
   }

   my $payload = $nrf->read_rx_payload( $plen )->get;
   printf "Payload: %v.02x\n", $payload;

   $nrf->flush_rx_fifo->get;
   $nrf->reset_interrupt->get;
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

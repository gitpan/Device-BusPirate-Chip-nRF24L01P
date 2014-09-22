#!/usr/bin/perl

use strict;
use warnings;

use Device::BusPirate;
use Getopt::Long;
use Data::Dump 'pp';

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
   PRIM_RX => 0,
   RF_DR   => "2E6",  # 2 Mbit/sec,
   RF_CH   => $CHANNEL,
   ARD     => 1500,   # 1500usec retransmit delay
   ARC     => 9,
   AW      => $AW,
   TX_ADDR => $ADDRESS,
   EN_DPL  => $DPL,
)->get;

# We need to set pipe 0's RX ADDR to equal TX ADDR so we receive the auto-ack
$nrf->change_rx_config( 0,
   RX_ADDR => $ADDRESS,
)->get;

$nrf->clear_caches;
printf "PTX config:\n%s\n%s\n", pp($nrf->read_config->get), pp($nrf->read_rx_config( 0 )->get);

printf "Transmitting on channel %d address %s\n",
   @{ $nrf->read_config->get }{qw( RF_CH TX_ADDR )};

$nrf->flush_tx_fifo->get;

$nrf->reset_interrupt->get;

$nrf->pwr_up( 1 )->get;
print "PWR_UP\n";

$nrf->write_tx_payload( "X" )->get;

$nrf->chip_enable( 1 )->get;
print "CE high - entered PTX mode...\n";

my $status;
1 while $status = $nrf->read_status->get and
   not ( $status->{TX_DS} || $status->{MAX_RT} );

$nrf->chip_enable( 0 )->get;

if( $status->{MAX_RT} ) {
   print STDERR "MAX_RT exceeded; packet lost\n";
   printf "Observe TX: %s\n", pp($nrf->observe_tx_counts->get);
}
else {
   print "Packet sent\n";
}

$nrf->reset_interrupt->get;

END {
   if( $nrf ) {
      $nrf->chip_enable( 0 )->get;
      $nrf->pwr_up( 0 )->get;
      $nrf->power(0)->get;
      print "Power off\n";
   }
   $pirate and $pirate->stop;
}

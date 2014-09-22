#  You may distribute under the terms of either the GNU General Public License
#  or the Artistic License (the same terms as Perl itself)
#
#  (C) Paul Evans, 2014 -- leonerd@leonerd.org.uk

package Device::BusPirate::Chip::nRF24L01P;

use strict;
use warnings;
use 5.010;
use base qw( Device::BusPirate::Chip );

our $VERSION = '0.01';

use Carp;

use constant CHIP => "nRF24L01+";
use constant MODE => "SPI";

use constant DEFAULT_SPEED => "1M";

=head1 NAME

C<Device::BusPirate::Chip::nRF24L01P> - use an F<nRF24L01+> chip with C<Device::BusPirate>

=head1 DESCRIPTION

This L<Device::BusPirate::Chip> subclass provides specific communication to a
F<Nordic Semiconductor> F<nRF24L01+> chip attached to the F<Bus Pirate> via
SPI.

The reader is presumed to be familiar with the general operation of this chip;
the documentation here will not attempt to explain or define chip-specific
concepts or features, only the use of this module to access them.

=cut

sub new
{
   my $class = shift;
   my ( $bp, %opts ) = @_;

   my $self = $class->SUPER::new( @_ );

   $self->{$_} = $opts{$_} for qw( open_drain speed );

   $self->{registers} = []; # cache of the values we write to config registers

   return $self;
}

sub mount
{
   my $self = shift;
   my ( $mode ) = @_;

   $self->SUPER::mount( $mode )
      ->then( sub {
         $mode->configure(
            open_drain => $self->{open_drain},
            speed      => $self->{speed} // DEFAULT_SPEED,
         );
      })
}

=head1 METHODS

=cut

# Commands
use constant {
   CMD_R_REGISTER          => 0x00,
   CMD_W_REGISTER          => 0x20,
   CMD_R_RX_PAYLOAD        => 0x61,
   CMD_W_TX_PAYLOAD        => 0xA0,
   CMD_FLUSH_TX            => 0xE1,
   CMD_FLUSH_RX            => 0xE2,
   CMD_REUSE_TX_PL         => 0xE3,
   CMD_R_RX_PL_WID         => 0x60,
   CMD_W_ACK_PAYLOAD       => 0xA8,
   CMD_W_TX_PAYLOAD_NO_ACK => 0xB0,
   CMD_NOP                 => 0xFF,
};

# Register numbers and lengths, and bitfields
use constant {
   REG_CONFIG      => [ 0x00, 1 ],
      MASK_RX_RD      => 1<<6,
      MASK_TX_DS      => 1<<5,
      MASK_MAX_RT     => 1<<4,
      EN_CRC          => 1<<3,
      CRCO            => 1<<2,
      PWR_UP          => 1<<1,
      PRIM_RX         => 1<<0,
   REG_EN_AA       => [ 0x01, 1 ],
      # per-pipe bitmask
   REG_EN_RXADDR   => [ 0x02, 1 ],
      # per-pipe bitmask
   REG_SETUP_AW    => [ 0x03, 1 ],
      # int
   REG_SETUP_RETR  => [ 0x04, 1 ],
      ARD             => 0x0f<<4,
      ARC             => 0x0f,
   REG_RF_CH       => [ 0x05, 1 ],
      # int
   REG_RF_SETUP    => [ 0x06, 1 ],
      CONT_WAVE       => 1<<7,
      RF_DR_LOW       => 1<<5,
      PLL_LOCK        => 1<<4,
      RF_DR_HIGH      => 1<<3,
      RF_PWR          => 3<<1,
   REG_STATUS      => [ 0x07, 1 ],
      RX_DR           => 1<<6,
      TX_DS           => 1<<5,
      MAX_RT          => 1<<4,
      RX_P_NO         => 7<<1,
      TX_FULL_STAT    => 1<<0,
   REG_OBSERVE_TX  => [ 0x08, 1 ],
      PLOS_CNT        => 0x0f<<4,
      ARC_CNT         => 0x0f,
   REG_RPD         => [ 0x09, 1 ],
      # bool
   REG_RX_ADDR_P0  => [ 0x0A, 5 ],
   REG_RX_ADDR_P1  => [ 0x0B, 5 ],
   REG_RX_ADDR_P2  => [ 0x0C, 1 ],
   REG_RX_ADDR_P3  => [ 0x0D, 1 ],
   REG_RX_ADDR_P4  => [ 0x0E, 1 ],
   REG_RX_ADDR_P5  => [ 0x0F, 1 ],
   REG_TX_ADDR     => [ 0x10, 5 ],
      # addresses
   REG_RX_PW_P0    => [ 0x11, 1 ],
   REG_RX_PW_P1    => [ 0x12, 1 ],
   REG_RX_PW_P2    => [ 0x13, 1 ],
   REG_RX_PW_P3    => [ 0x14, 1 ],
   REG_RX_PW_P4    => [ 0x15, 1 ],
   REG_RX_PW_P5    => [ 0x16, 1 ],
      # ints
   REG_FIFO_STATUS => [ 0x17, 1 ],
      TX_REUSE        => 1<<6,
      TX_FULL         => 1<<5,
      TX_EMPTY        => 1<<4,
      RX_FULL         => 1<<1,
      RX_EMPTY        => 1<<0,
   REG_DYNPD       => [ 0x1C, 1 ],
      # per-pipe bitmask
   REG_FEATURE     => [ 0x1D, 1 ],
      EN_DPL          => 1<<2,
      EN_ACK_PAY      => 1<<1,
      EN_DYN_ACK      => 1<<0,
};

=head2 $nrf->clear_caches

The chip object stores a cache of the register values it last read or wrote,
so it can optimise updates of configuration. This method clears these caches,
ensuring a fresh SPI transfer next time the register needs to be read.

This should not normally be necessary, other than for debugging.

=cut

sub clear_caches
{
   my $self = shift;
   undef @{ $self->{registers} };
}

=head2 $status = $nrf->latest_status

Returns the latest cached copy of the status register from the most recent SPI
interaction. As this method does not perform any IO, it returns its result
immediately rather than via a Future.

Returns a HASH reference containing the following boolean fields

 RX_DR TX_DS MAX_RT TX_FULL

Also returned is a field called C<RX_P_NO>, which is either a pipe number (0
to 5) or undef.

=cut

sub _decode_status
{
   my ( $status ) = @_;

   my $rx_p_no = ( $status & RX_P_NO ) >> 1;
   undef $rx_p_no if $rx_p_no > 5;

   return {
      RX_DR   => !!( $status & RX_DR  ),
      TX_DS   => !!( $status & TX_DS  ),
      MAX_RT  => !!( $status & MAX_RT ),
      RX_P_NO => $rx_p_no,
      TX_FULL => !!( $status & TX_FULL_STAT ), # different mask to FIFO_STATUS
   }
}

sub latest_status
{
   my $self = shift;
   return _decode_status $self->{latest_status};
}

=head2 $nrf->reset_interrupt->get

Clears the interrupt flags in the C<STATUS> register.

=cut

sub reset_interrupt
{
   my $self = shift;

   $self->_write_register_volatile( REG_STATUS, chr( RX_DR | TX_DS | MAX_RT ) );
}

sub _do_command
{
   my $self = shift;
   my ( $cmd, $data ) = @_;
   $data //= "";

   $self->mode->writeread_cs( chr( $cmd ) . $data )->then( sub {
      my ( $buf ) = @_;
      $self->{latest_status} = ord substr $buf, 0, 1, "";
      Future->done( $buf );
   });
}

=head2 $status = $nrf->read_status->get

Reads and returns the current content of the status register as a HASH
reference as per C<latest_status>.

=cut

sub read_status
{
   my $self = shift;
   $self->_do_command( CMD_NOP )->then( sub {
      Future->done( $self->latest_status )
   });
}

# Always performs an SPI operation
sub _read_register_volatile
{
   my $self = shift;
   my ( $reg ) = @_;

   my ( $regnum, $len ) = @$reg;

   $self->_do_command( CMD_R_REGISTER | $regnum, ( "\0" x $len ) )
      ->on_done( sub {
         $self->{registers}[$regnum] = $_[0];
      });
}

# Returns the cached value if present
sub _read_register
{
   my $self = shift;
   my ( $reg ) = @_;

   my ( $regnum ) = @$reg;

   defined $self->{registers}[$regnum] ?
      Future->done( $self->{registers}[$regnum] ) :
      $self->_read_register_volatile( $reg );
}

# Always performs an SPI operation
sub _write_register_volatile
{
   my $self = shift;
   my ( $reg, $data ) = @_;

   my ( $regnum, $len ) = @$reg;
   $len == length $data or croak "Attempted to write the wrong length";

   $self->_do_command( CMD_W_REGISTER | $regnum, $data )
      ->then( sub {
         $self->{registers}[$regnum] = $data;
         Future->done()
      });
}

# Doesn't bother if no change
sub _write_register
{
   my $self = shift;
   my ( $reg, $data ) = @_;

   my ( $regnum ) = @$reg;

   return Future->done() if
      defined $self->{registers}[$regnum] and $self->{registers}[$regnum] eq $data;

   return $self->_write_register_volatile( $reg, $data );
}

=head2 $config = $nrf->read_config->get

=head2 $nrf->change_config( %config )->get

Reads or writes the chip-wide configuration. This is an amalgamation of all
the non-pipe-specific configuration registers; C<CONFIG>, C<SETUP_AW>,
C<SETUP_RETR>, C<RF_CH>, C<RF_SETUP>, C<TX_ADDR> and C<FEATURE>.

When reading, the fields are returned in a HASH reference whose names are the
original bitfield names found in the F<Nordic Semiconductor> data sheet. When
writing, these fields are accepted as named parameters to the C<change_config>
method directly.

Some of the fields have special processing for convenience. They are:

=over 4

=item * CRCO

Gives the CRC length in bytes, as either 1 or 2.

=item * AW

Gives the full address width in bytes, between 3 and 5.

=item * ARD

Gives the auto retransmit delay in microseconds directly; a multiple of 250
between 250 and 4000.

=item * RF_DR

Gives the RF data rate in bytes/sec; omits the C<RF_DR_LOW> and C<RF_DR_HIGH>
fields; as 250000, 1000000 or 2000000

=item * RF_PWR

Gives the RF output power in dBm directly, as -18, -12, -6 or 0.

=item * TX_ADDR

Gives the PTX address as a string of 5 capital hexadecimal encoded octets,
separated by colons.

=back

Whenever the config is read it is cached within the C<$chip> instance.
Whenever it is written, any missing fields in the passed configuration are
pre-filled by the cached config, and only those registers that need writing
will be written.

=cut

my @CRCOs   = ( 1, 2 );
my @AWs     = ( undef, 3, 4, 5 );
my @ARDs    = map { ( $_ + 1 ) * 250 } 0 .. 15;
my @RF_DRs  = ( 1E6, 2E6, 250E3, undef );
my @RF_PWRs = ( -18, -12, -6, 0 );

sub _idx_of
{
   my $want = shift;
   foreach my $idx ( 0 .. $#_ ) {
      defined $_[$idx] and $_[$idx] == $want and return $idx;
   }
   return undef;
}

sub _unpack_addr
{
   my ( $addr ) = @_;
   return join ":", map { sprintf "%02X", ord } split //, $addr;
}

sub _pack_addr
{
   my ( $addr ) = @_;
   return join "", map { chr hex } split m/:/, $addr;
}

sub _unpack_config
{
   my %regs = @_;

   my $config     = $regs{config};
   my $setup_retr = $regs{setup_retr};
   my $rf_setup   = $regs{rf_setup};
   my $feature    = $regs{feature};

   return
      # REG_CONFIG
      MASK_RX_RD  => !!( $config & MASK_RX_RD ),
      MASK_TX_DS  => !!( $config & MASK_TX_DS ),
      MASK_MAX_RT => !!( $config & MASK_MAX_RT ),
      EN_CRC      => !!( $config & EN_CRC ),
      CRCO        => $CRCOs[!!( $config & CRCO )],
      PWR_UP      => !!( $config & PWR_UP ),
      PRIM_RX     => !!( $config & PRIM_RX ),

      # REG_SETUP_AW
      AW          => $AWs[ $regs{setup_aw} & 0x03 ],

      # REG_SETUP_RETR
      ARD         => $ARDs[( $setup_retr & ARD ) >> 4],
      ARC         =>       ( $setup_retr & ARC ),

      # REG_RF_CH
      RF_CH       => $regs{rf_ch},

      # REG_RF_SETUP
      CONT_WAVE   => !!( $rf_setup & CONT_WAVE ),
      PLL_LOCK    => !!( $rf_setup & PLL_LOCK ),
      RF_DR       => do {
         my $rf_dr = 2*!!( $rf_setup & RF_DR_LOW ) + !!( $rf_setup & RF_DR_HIGH );
         $RF_DRs[$rf_dr] },
      RF_PWR      => $RF_PWRs[( $rf_setup & RF_PWR ) >> 1],

      # REG_TX_ADDR
      TX_ADDR     => _unpack_addr( $regs{tx_addr} ),

      # REG_FEATURE
      EN_DPL      => !!( $feature & EN_DPL ),
      EN_ACK_PAY  => !!( $feature & EN_ACK_PAY ),
      EN_DYN_ACK  => !!( $feature & EN_DYN_ACK ),
}

sub _pack_config
{
   my %config = @_;

   return
      config => (
         ( $config{MASK_RX_RD}  ? MASK_RX_RD  : 0 ) |
         ( $config{MASK_TX_DS}  ? MASK_TX_DS  : 0 ) |
         ( $config{MASK_MAX_RT} ? MASK_MAX_RT : 0 ) |
         ( $config{EN_CRC}      ? EN_CRC      : 0 ) |
         ( ( _idx_of $config{CRCO}, @CRCOs ) // croak "Unsupported 'CRCO'" ) * CRCO |
         ( $config{PWR_UP}      ? PWR_UP      : 0 ) |
         ( $config{PRIM_RX}     ? PRIM_RX     : 0 ) ),

      setup_aw => (
         ( _idx_of $config{AW}, @AWs ) // croak "Unsupported 'AW'" ),

      setup_retr => (
         ( ( _idx_of $config{ARD}, @ARDs ) // croak "Unsupported 'ARD'" ) << 4 |
             $config{ARC} ),

      rf_ch => $config{RF_CH},

      rf_setup => do {
         my $rf_dr = ( _idx_of $config{RF_DR}, @RF_DRs ) // croak "Unsupported 'RF_DR'";

         ( $config{CONT_WAVE} ? CONT_WAVE  : 0 ) |
         ( ( $rf_dr & 2 )     ? RF_DR_LOW  : 0 ) |
         ( $config{PLL_LOCK}  ? PLL_LOCK   : 0 ) |
         ( ( $rf_dr & 1 )     ? RF_DR_HIGH : 0 ) |
         ( ( _idx_of $config{RF_PWR}, @RF_PWRs ) // croak "Unsupported 'RF_PWR'" ) << 1
      },

      tx_addr => _pack_addr( $config{TX_ADDR} ),

      feature => (
         ( $config{EN_DPL}     ? EN_DPL     : 0 ) |
         ( $config{EN_ACK_PAY} ? EN_ACK_PAY : 0 ) |
         ( $config{EN_DYN_ACK} ? EN_DYN_ACK : 0 ) );
}

sub read_config
{
   my $self = shift;

   Future->needs_all(
      map { $self->_read_register( $_ ) }
         REG_CONFIG, REG_SETUP_AW, REG_SETUP_RETR, REG_RF_CH, REG_RF_SETUP, REG_TX_ADDR, REG_FEATURE,
   )->then( sub {
      $_ = ord $_ for @_[0,1,2,3,4,6]; # [5] is TX_ADDR
      my %regs;
      @regs{qw( config setup_aw setup_retr rf_ch rf_setup tx_addr feature )} = @_;

      Future->done( { _unpack_config %regs } );
   });
}

sub change_config
{
   my $self = shift;
   my %changes = @_;

   $self->read_config
   ->then( sub {
      my ( $config ) = @_;
      my %new_registers = _pack_config %$config, %changes;

      my @f;
      foreach (qw( config setup_aw setup_retr rf_ch rf_setup feature )) {
         push @f, $self->_write_register( $self->${\"REG_\U$_"}, chr $new_registers{$_} );
      }
      push @f, $self->_write_register( REG_TX_ADDR, $new_registers{tx_addr} );

      Future->needs_all( @f )
   })->then_done();
}

=head2 $config = $nrf->read_rx_config( $pipeno )->get

=head2 $nrf->change_rx_config( $pipeno, %config )->get

Reads or writes the per-pipe RX configuration. This is composed of the
per-pipe bits of the C<EN_AA> and C<EN_RXADDR> registers and its
C<RX_ADDR_Pn> register.

Addresses are given as a string of 5 octets in capitalised hexadecimal
notation, separated by colons.

When reading an address from pipes 2 to 5, the address of pipe 1 is used to
build a complete address string to return. When writing and address to these
pipes, all but the final byte is ignored.

=cut

sub read_rx_config
{
   my $self = shift;
   my ( $pipeno ) = @_;

   $pipeno >= 0 and $pipeno < 6 or croak "Invalid pipe number $pipeno";
   my $mask = 1 << $pipeno;

   Future->needs_all(
      map { $self->_read_register( $_ ) }
         REG_EN_AA, REG_EN_RXADDR, REG_DYNPD, # bitwise
         $self->${\"REG_RX_PW_P$pipeno"}, $self->${\"REG_RX_ADDR_P$pipeno"},
         # Pipes 2 to 5 share the first 4 octects of PIPE1's address
         ( $pipeno >= 2 ? REG_RX_ADDR_P1 : () ),
   )->then( sub {
      my ( $en_aa, $en_rxaddr, $dynpd, $width, $addr, $p1addr ) = @_;
      $_ = ord $_ for $en_aa, $en_rxaddr, $dynpd, $width;

      $addr = substr( $p1addr, 0, 4 ) . $addr if $pipeno >= 2;

      Future->done( {
         EN_AA     => !!( $en_aa     & $mask ),
         EN_RXADDR => !!( $en_rxaddr & $mask ),
         DYNPD     => !!( $dynpd     & $mask ),
         RX_PW     => $width,
         RX_ADDR   => _unpack_addr $addr,
      } );
   });
}

sub change_rx_config
{
   my $self = shift;
   my ( $pipeno, %changes ) = @_;

   $pipeno >= 0 and $pipeno < 6 or croak "Invalid pipe number $pipeno";
   my $mask = 1 << $pipeno;

   my $REG_RX_PW_Pn   = $self->${\"REG_RX_PW_P$pipeno"};
   my $REG_RX_ADDR_Pn = $self->${\"REG_RX_ADDR_P$pipeno"};

   Future->needs_all(
      map { $self->_read_register( $_ ) }
         REG_EN_AA, REG_EN_RXADDR, REG_DYNPD, $REG_RX_PW_Pn, $REG_RX_ADDR_Pn
   )->then( sub {
      my ( $en_aa, $en_rxaddr, $dynpd, $width, $addr ) = @_;
      $_ = ord $_ for $en_aa, $en_rxaddr, $dynpd, $width;

      if( exists $changes{EN_AA} ) {
         $en_aa &= ~$mask;
         $en_aa |=  $mask if $changes{EN_AA};
      }
      if( exists $changes{EN_RXADDR} ) {
         $en_rxaddr &= ~$mask;
         $en_rxaddr |=  $mask if $changes{EN_RXADDR};
      }
      if( exists $changes{DYNPD} ) {
         $dynpd &= ~$mask;
         $dynpd |=  $mask if $changes{DYNPD};
      }
      if( exists $changes{RX_PW} ) {
         $width = $changes{RX_PW};
      }
      if( exists $changes{RX_ADDR} ) {
         $addr = _pack_addr $changes{RX_ADDR};
         $addr = substr( $addr, -1 ) if $pipeno >= 2;
      }

      Future->needs_all(
         $self->_write_register( REG_EN_AA,     chr $en_aa ),
         $self->_write_register( REG_EN_RXADDR, chr $en_rxaddr ),
         $self->_write_register( REG_DYNPD,     chr $dynpd ),
         $self->_write_register( $REG_RX_PW_Pn, chr $width ),
         $self->_write_register( $REG_RX_ADDR_Pn, $addr ),
      );
   })->then_done();
}

=head2 $counts = $nrf->observe_tx_counts->get

Reads the C<OBSERVE_TX> register and returns the two counts from it.

=cut

sub observe_tx_counts
{
   my $self = shift;

   $self->_read_register_volatile( REG_OBSERVE_TX )->then( sub {
      my ( $buf ) = @_;
      $buf = ord $buf;

      Future->done( {
         PLOS_CNT => ( $buf & PLOS_CNT ) >> 4,
         ARC_CNT  => ( $buf & ARC_CNT ),
      } );
   });
}

=head2 $rpd = $nrf->rpd->get

Reads the C<RPD> register

=cut

sub rpd
{
   my $self = shift;

   $self->_read_register_volatile( REG_RPD )->then( sub {
      my ( $buf ) = @_;
      $buf = ord $buf;

      Future->done( $buf & 1 );
   });
}

=head2 $status = $nrf->fifo_status->get

Reads the C<FIFO_STATUS> register and returns the five bit fields from it.

=cut

sub fifo_status
{
   my $self = shift;

   $self->_read_register_volatile( REG_FIFO_STATUS )->then( sub {
      my ( $buf ) = @_;
      $buf = ord $buf;

      Future->done( {
         TX_REUSE => !!( $buf & TX_REUSE ),
         TX_FULL  => !!( $buf & TX_FULL ),
         TX_EMPTY => !!( $buf & TX_EMPTY ),

         RX_FULL  => !!( $buf & RX_FULL ),
         RX_EMPTY => !!( $buf & RX_EMPTY ),
      } );
   });
}

=head2 $nrf->pwr_up( $pwr )->get

A convenient shortcut to setting the C<PWR_UP> configuration bit.

=cut

sub pwr_up
{
   my $self = shift;
   $self->change_config( PWR_UP => shift );
}

=head2 $nrf->chip_enable( $ce )->get

A wrapper around the underlying C<aux> method. This presumes that the C<CE>
pin is attached to the C<AUX> F<Bus Pirate> line.

=cut

sub chip_enable
{
   my $self = shift;
   $self->aux( @_ );
}

=head2 $len = $nrf->read_rx_payload_width->get

Returns the width of the most recently received payload, when in C<DPL> mode.
Remember that C<DPL> needs to be enabled (using C<EN_DPL>) on both the
transmitter and receiver before this will work.

=cut

sub read_rx_payload_width
{
   my $self = shift;

   $self->_do_command( CMD_R_RX_PL_WID, "\0" )->then( sub {
      my ( $buf ) = @_;
      Future->done( ord $buf );
   });
}

=head2 $data = $nrf->read_rx_payload( $len )->get

Reads the most recently received RX FIFO payload buffer.

=cut

sub read_rx_payload
{
   my $self = shift;
   my ( $len ) = @_;

   $len > 0 and $len <= 32 or croak "Invalid RX payload length $len";

   $self->_do_command( CMD_R_RX_PAYLOAD, "\0" x $len )
}

=head2 $nrf->write_tx_payload( $data, %opts )->get

Writes the next TX FIFO payload buffer. Takes the following options:

=over 4

=item no_ack => BOOL

If true, uses the C<W_TX_PAYLOAD_NO_ACK> command, requesting that this payload
does not requre auto-ACK.

=back

=cut

sub write_tx_payload
{
   my $self = shift;
   my ( $data, %opts ) = @_;

   my $len = length $data;
   $len > 0 and $len <= 32 or croak "Invalid TX payload length $len";

   my $cmd = $opts{no_ack} ? CMD_W_TX_PAYLOAD_NO_ACK : CMD_W_TX_PAYLOAD;

   $self->_do_command( $cmd, $data )
      ->then_done();
}

=head2 $nrf->flush_rx_fifo->get

=head2 $nrf->flush_tx_fifo->get

Flush the RX or TX FIFOs, discarding all their contents.

=cut

sub flush_rx_fifo
{
   my $self = shift;

   $self->_do_command( CMD_FLUSH_RX )
      ->then_done();
}

sub flush_tx_fifo
{
   my $self = shift;

   $self->_do_command( CMD_FLUSH_TX )
      ->then_done();
}

=head1 AUTHOR

Paul Evans <leonerd@leonerd.org.uk>

=cut

0x55AA;

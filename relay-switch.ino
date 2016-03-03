#include <limits.h>
#include <stdint.h>
#include <SPI.h>
#include <RF24.h>

/*
 * Set the following to 1 to run debug logic and output debug information to the
 * serial output.
 */
#define DEBUG 0

// RF24 RADIO
#define RADIO_CHANNEL 5

// DIGITAL PINS
#define PIN_MASTER_JUMPER 3
#define PIN_FAN_SWITCH 4
#define PIN_LIGHT_SWITCH 5
#define PIN_FAN_RELAY 6
#define PIN_LIGHT_RELAY 7

// SYNCHRONIZATION
#define HEARTBEAT_TIMEOUT 1000
#define HEARTBEAT 500

// DEBUGGING
#define TRACE( x ) do { if ( DEBUG ) { x } } while ( 0 )
#define PRINT( x ) ( Serial.print( x ) )
#define PRINTLN( x ) ( Serial.println( x ) )

/* -------------------------------------------------------------------------- */

/*
 * Represents a linked list of communication devices (Arduinos with radios).
 *
 * device_id - represents the unique communication id of the radio device.
 * seq_num   - represents a unique and increasing number across consecutive
 *             transmission of packets for this device. This value should be
 *             updated if a packet with a larger seq_num is received for this
 *             device_id. If a packet is received with a smaller seq_num, then
 *             it should be ignored.
 * next      - represents the next device in the the linked list of devices that
 *             have transmitted packets. NULL if there is no such next device.
 */
typedef struct device_t {
  unsigned long id;
  unsigned long seq_num;
  device_t * next;
} device_t;

/*
 * Represents the type of data being transmitted in a packet_t. The MASTER only
 * transmits SYNC packets. Only clients transmit FAN_FLIPPED and LIGHT_FLIPPED
 * packets.
 */
typedef enum packet_type_t {
  SYNC = 1,
  FAN_FLIPPED = 2,
  LIGHT_FLIPPED = 3
} packet_type_t;

/*
 * Data structure used for communicating with other radio devices.
 *
 * device_id - represents the unique communication id of a radio device.
 * seq_num   - represents a unique and increasing number across consecutive
 *             transmission of packets for this device.
 * type      - represents the type of information carried by the packet. the fan
 *             and light fields are only used for SYNC packets.
 * fan       - represents the current fan state for SYNC packets.
 * light     - represents the current light state for SYNC packets.
 */
typedef struct packet_t {
  unsigned long device_id;
  unsigned long seq_num;
  packet_type_t type;
  bool fan;
  bool light;
} packet_t;

/*
 * Represents a physical switch. Call update() to update the switch's state. The
 * switch's state is either flipped or not. A switch is flipped when, during two
 * consecutive calls to update(), the state of the switch (up or down) changes.
 */
class Switch {

    const int pin;
    bool up;
    bool flipped;

  public:

    Switch( int pin, bool flipped = false ) : pin( pin ), flipped( flipped ) {
      pinMode( pin, INPUT_PULLUP );
      up = digitalRead( pin );
    }

    /*
     * Updates switch position and flipped state.
     */
    inline void update() {
      bool up = digitalRead( pin );
      flipped = this->up != up;
      this->up = up;
    }

    /**
     * Returns whether or not the switch was flipped during two consecutive
     * calls to update().
     */
    inline bool isFlipped() const {
      return flipped;
    }

};

/*
 * Represents a physical relay. Call update() to update the relay's state. The
 * relay's state is either on or off. A relay can be switched on or off by
 * calling isOn( true ) or isOn( false ), respectively.
 */
class Relay {

    const int pin;
    bool on;

  public:

    Relay( int pin, bool on = false ) : pin( pin ), on( on ) {
      pinMode( pin, OUTPUT );
      update();
    }

    /*
     * Updates relay state.
     */
    inline void update() const {
      digitalWrite( pin, on );
    }

    /*
     * Returns whether or not the relay is on.
     */
    inline bool isOn() const {
      return on;
    }

    /*
     * Sets whether or not the relay is on.
     */
    inline void isOn( bool on ) {
      this->on = on;
    }

    /*
     * Toggles relay state and returns whether or not the relay is on.
     */
    inline bool toggle() {
      return this->on = !this->on;
    }

};

/* -------------------------------------------------------------------------- */

// Variables used for synchronization
bool MASTER = false;
int heartbeat_timeout;
int heartbeat;

// Variables used describe and control external components
Switch * fan_switch, * light_switch;
Relay * fan_relay, * light_relay;

// Variables used for communication
unsigned long device_id;
unsigned long seq_num;
device_t * devices;
packet_t packet;

// Variables for radio
RF24 radio( 2, 3 );  // TODO: CONFIRM RF24( 2, 3 ) IS CORRECT
const uint64_t pipes[2] = { 0xe7e7e7e7e7LL, 0xc2c2c2c2c2LL };

/* -------------------------------------------------------------------------- */

inline device_t * addDevice( unsigned long id ) {

  TRACE( PRINTLN( "Adding device." ); );

  device_t * d = ( device_t * ) malloc( sizeof( device_t ) );

  d->id = id;
  d->seq_num = 0;
  d->next = devices;
  devices = d;

  return d;

}

inline device_t * getDevice( unsigned long id ) {

  if ( !id ) return NULL;

  device_t * curr = devices;
  while ( curr ) {
    if ( curr->id == id ) return curr;
    curr = curr->next;
  }

  return addDevice( id );

}

inline void printPacket() {
  PRINT( "device_id: " );
  PRINTLN( packet.device_id );
  PRINT( "seq_num: " );
  PRINTLN( packet.seq_num );
  PRINT( "type: " );
  PRINTLN( packet.type );
  PRINT( "locked: " );
  PRINTLN( packet.locked );
}

inline void xmitPacket() {

  TRACE(
    PRINTLN( "Sending packet." );
    printPacket();
  );

  // Switch to xmit mode
  radio.openWritingPipe( pipes[ 1 ] );
  radio.openReadingPipe( 1, pipes[ 0 ] );
  radio.stopListening();

  // Send packet
  radio.write( &packet, sizeof( packet_t ) );

  // Switch back to recv mode
  radio.openWritingPipe( pipes[ 0 ] );
  radio.openReadingPipe( 1, pipes[ 1 ] );
  radio.startListening();

  delay( 1 );

}

inline bool recvPacket() {
  if ( radio.available() ) {

    // Get packet
    radio.read( &packet, sizeof( packet_t ) );

    TRACE( PRINTLN( "Received packet." ); );

    // Get associated device
    device_t * device = getDevice( packet.device_id );

    // Only accept new packets from valid devices
    if ( device && packet.seq_num > device->seq_num ) {
      TRACE( printPacket(); );
      device->seq_num = packet.seq_num;
      return true;
    }

  }
  return false;
}

/* -------------------------------------------------------------------------- */

void setup() {

  TRACE(
    Serial.begin( 9600 );
    delay( 2000 );
  );

  // Determine MASTER or CLIENT
  pinMode( PIN_MASTER_JUMPER, INPUT_PULLUP );
  MASTER = !digitalRead( PIN_MASTER_JUMPER );

  // Generate communication parameters
  randomSeed( analogRead( 0 ) );
  device_id = random( LONG_MAX );
  seq_num = random( 1L << ( sizeof( seq_num ) * 4 ) );

  // Configure heartbeat
  heartbeat_timeout = 0;
  heartbeat = HEARTBEAT;

  // Configure switches and controls
  fan_switch = new Switch( PIN_FAN_SWITCH );
  light_switch = new Switch( PIN_LIGHT_SWITCH );
  fan_relay = new Relay( PIN_FAN_RELAY );
  light_relay = new Relay( PIN_LIGHT_RELAY );

  // Configure RF24 radio
  radio.begin();
  radio.setChannel( RADIO_CHANNEL );
//  radio.setPayloadSize( sizeof( packet_t ) );  // TODO: IS THIS NEEDED?
//  radio.setCRCLength( RF24_CRC_8 );            // TODO: IS THIS NEEDED?
  radio.openWritingPipe( pipes[ 0 ] );
  radio.openReadingPipe( 1, pipes[ 1 ] );
  radio.startListening();

}

/* -------------------------------------------------------------------------- */

void loop() {

  // Handle heartbeats
  if ( MASTER ) {
    if ( heartbeat ) {
      heartbeat--;
    } else {

      TRACE( PRINT( "Sending heartbeat sync." ); );

      // Send heartbeat sync packet
      packet.device_id = device_id;
      packet.seq_num = seq_num++;
      packet.type = SYNC;
      packet.fan = fan_relay->isOn();
      packet.light = light_relay->isOn();

      xmitPacket();

      // Reset the counter
      heartbeat = HEARTBEAT;

    }
  } else if ( heartbeat_timeout == HEARTBEAT_TIMEOUT ) {

    // We haven't heard from server for long enough; turn off relays to save power
    fan_relay->isOn( false );
    light_relay->isOn( false );

    // Reset the counter
    heartbeat_timeout = 0;

    delay( 1 );
    return;

  } else {
    heartbeat_timeout++;
  }

  // Handle inbound packets
  if ( recvPacket() ) {
    if ( MASTER ) {

      if ( packet.type == FAN_FLIPPED ) {

        // MASTER updates internal state and tells clients to SYNC to new state.
        packet.device_id = device_id;
        packet.seq_num = seq_num++;
        packet.type = SYNC;
        packet.fan = fan_relay->toggle();
        packet.light = light_relay->toggle();

        xmitPacket();

      } else if ( packet.type == LIGHT_FLIPPED ) {

        // MASTER updates internal state and tells clients to SYNC to new state.
        packet.device_id = device_id;
        packet.seq_num = seq_num++;
        packet.type = SYNC;
        packet.fan = fan_relay->toggle();
        packet.light = light_relay->toggle();

        xmitPacket();

      } else {
        TRACE( PRINTLN( "Received unknown packet type." ); );
      }

    } else if ( packet.type == SYNC ) {

      // Set relay states according to SYNC packet
      fan_relay->isOn( packet.fan );
      light_relay->isOn( packet.light );

      // Reset the heartbeat
      heartbeat = HEARTBEAT;
      heartbeat_timeout = 0;

    } else {
      TRACE( PRINTLN( "Received invalid packet." ); );
    }
  }

  // Update switches
  fan_switch->update();
  light_switch->update();

  // Handle switches
  if ( MASTER ) {
    if ( fan_switch->isFlipped() || light_switch->isFlipped() ) {

      // MASTER updates internal state and tells clients to SYNC to new state.
      packet.device_id = device_id;
      packet.seq_num = seq_num++;
      packet.type = SYNC;
      packet.fan = fan_relay->toggle();
      packet.light = light_relay->toggle();

      xmitPacket();

    }
  } else {

    if ( fan_switch->isFlipped() ) {

      // Clients tell MASTER about flipped switch
      packet.device_id = device_id;
      packet.seq_num = seq_num++;
      packet.type = FAN_FLIPPED;

      xmitPacket();

    }

    if ( light_switch->isFlipped() ) {

      // Clients tell MASTER about flipped switch
      packet.device_id = device_id;
      packet.seq_num = seq_num++;
      packet.type = LIGHT_FLIPPED;

      xmitPacket();

    }

  }

  // Update relays
  fan_relay->update();
  light_relay->update();


}

#define icon_width 45
#define icon_height 45

static const unsigned char steam_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x31, 0x06,
   0x00, 0x00, 0x00, 0xc0, 0x18, 0x03, 0x00, 0x00, 0x00, 0x60, 0x8c, 0x01,
   0x00, 0x00, 0x00, 0x60, 0x8c, 0x01, 0x00, 0x00, 0x00, 0x60, 0x8c, 0x01,
   0x00, 0x00, 0x00, 0xc0, 0x18, 0x03, 0x00, 0x00, 0x00, 0x80, 0x31, 0x06,
   0x00, 0x00, 0xf8, 0x87, 0x31, 0x06, 0x00, 0x00, 0xff, 0x1f, 0x31, 0x06,
   0x00, 0xc0, 0x03, 0x78, 0x18, 0x03, 0x00, 0xf0, 0x00, 0xe0, 0x89, 0x01,
   0x00, 0x38, 0x00, 0x80, 0x83, 0x01, 0x00, 0x1c, 0x00, 0x00, 0x87, 0x01,
   0x00, 0x0c, 0x00, 0x00, 0x0e, 0x03, 0x00, 0x06, 0x00, 0x00, 0x0c, 0x02,
   0x00, 0x03, 0x06, 0x0c, 0x18, 0x00, 0x00, 0x03, 0x0f, 0x1e, 0x18, 0x00,
   0x80, 0x01, 0x0f, 0x1e, 0x30, 0x00, 0x80, 0x01, 0x0f, 0x1e, 0x30, 0x00,
   0x80, 0x00, 0x0f, 0x1e, 0x20, 0x00, 0xc0, 0x00, 0x06, 0x0c, 0x60, 0x00,
   0xc0, 0x00, 0x00, 0x00, 0x60, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x60, 0x00,
   0xc0, 0x00, 0x00, 0x00, 0x60, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x60, 0x00,
   0xc0, 0x00, 0x00, 0x00, 0x60, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x60, 0x00,
   0x80, 0x10, 0x00, 0x00, 0x61, 0x00, 0x80, 0x31, 0x00, 0x80, 0x31, 0x00,
   0x80, 0x61, 0x00, 0xc0, 0x31, 0x00, 0x00, 0xe3, 0x00, 0xe0, 0x30, 0x00,
   0x00, 0xc3, 0x01, 0x70, 0x18, 0x00, 0x00, 0xe6, 0x07, 0x3c, 0x0c, 0x00,
   0x00, 0x6e, 0xff, 0x1f, 0x0c, 0x00, 0x00, 0x3c, 0xff, 0x07, 0x06, 0x00,
   0x00, 0x38, 0x73, 0x80, 0x03, 0x00, 0x00, 0x98, 0x33, 0xc0, 0x01, 0x00,
   0x00, 0x98, 0x19, 0xf0, 0x00, 0x00, 0x00, 0x98, 0xf9, 0x3f, 0x00, 0x00,
   0x00, 0x98, 0xfc, 0x07, 0x00, 0x00, 0x00, 0x18, 0x0c, 0x00, 0x00, 0x00,
   0x00, 0xf0, 0x07, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const unsigned char steam_rotate_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x8c, 0x01,
   0x00, 0x00, 0x00, 0xc0, 0x18, 0x03, 0x00, 0x00, 0x00, 0x80, 0x31, 0x06,
   0x00, 0x00, 0x00, 0x80, 0x31, 0x06, 0x00, 0x00, 0x00, 0x80, 0x31, 0x06,
   0x00, 0x00, 0x00, 0xc0, 0x18, 0x03, 0x00, 0x00, 0x00, 0x40, 0x8c, 0x01,
   0x00, 0x00, 0xf8, 0x07, 0x8c, 0x01, 0x00, 0x00, 0xff, 0x3f, 0x8c, 0x01,
   0x00, 0xc0, 0x03, 0x78, 0x18, 0x03, 0x00, 0xf0, 0x00, 0xe0, 0x11, 0x06,
   0x00, 0x38, 0x00, 0x80, 0x03, 0x06, 0x00, 0x1c, 0x00, 0x00, 0x07, 0x06,
   0x00, 0x0c, 0x00, 0x00, 0x0e, 0x03, 0x00, 0x06, 0x00, 0x00, 0x0c, 0x01,
   0x00, 0x03, 0x00, 0x00, 0x18, 0x00, 0x00, 0x83, 0x07, 0x3c, 0x18, 0x00,
   0x80, 0xc1, 0x0f, 0x7e, 0x30, 0x00, 0x80, 0x61, 0x1c, 0xc7, 0x30, 0x00,
   0x80, 0x70, 0x18, 0xc3, 0x21, 0x00, 0xc0, 0x20, 0x10, 0x81, 0x60, 0x00,
   0xc0, 0x00, 0x00, 0x00, 0x60, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x60, 0x00,
   0xc0, 0x00, 0x00, 0x00, 0x60, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x60, 0x00,
   0xc0, 0x00, 0x00, 0x00, 0x60, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x60, 0x00,
   0x80, 0x10, 0x00, 0x00, 0x61, 0x00, 0x80, 0x31, 0x00, 0x80, 0x31, 0x00,
   0x80, 0x61, 0x00, 0xc0, 0x31, 0x00, 0x00, 0xe3, 0x00, 0xe0, 0x30, 0x00,
   0x00, 0xe3, 0x01, 0x70, 0x18, 0x00, 0x00, 0xbe, 0x07, 0x3c, 0x0c, 0x00,
   0x00, 0x3e, 0xff, 0x1f, 0x0c, 0x00, 0x00, 0x98, 0xff, 0x07, 0x06, 0x00,
   0x00, 0x98, 0x71, 0x80, 0x03, 0x00, 0x00, 0x98, 0x31, 0xc0, 0x01, 0x00,
   0x00, 0x98, 0x1c, 0xf0, 0x00, 0x00, 0x00, 0x18, 0xfc, 0x3f, 0x00, 0x00,
   0x00, 0xf0, 0xff, 0x07, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const unsigned char outer_zone_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x0f, 0x00, 0x00,
   0x00, 0x00, 0xff, 0x7f, 0x00, 0x00, 0x00, 0xe0, 0x0f, 0xfc, 0x01, 0x00,
   0x00, 0xf0, 0x00, 0xc0, 0x07, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x0f, 0x00,
   0x00, 0x1e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x07, 0x00, 0x00, 0x38, 0x00,
   0x80, 0x03, 0x00, 0x00, 0x70, 0x00, 0xc0, 0x01, 0x00, 0x00, 0xe0, 0x00,
   0xc0, 0x00, 0x00, 0x00, 0xc0, 0x01, 0xe0, 0x00, 0x01, 0x20, 0x80, 0x01,
   0x60, 0x80, 0x03, 0x70, 0x80, 0x03, 0x70, 0xc0, 0x07, 0xf8, 0x00, 0x03,
   0x30, 0xc0, 0x07, 0xf8, 0x00, 0x07, 0x38, 0xc0, 0x07, 0xf8, 0x00, 0x06,
   0x18, 0xc0, 0x07, 0xf8, 0x00, 0x06, 0x18, 0x80, 0x03, 0x70, 0x00, 0x0e,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x18, 0x00, 0x00, 0x00, 0x00, 0x0c,
   0x1c, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x0c,
   0x1c, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x0c,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x18, 0x00, 0x00, 0x00, 0x00, 0x0e,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x06, 0x38, 0x00, 0x00, 0x00, 0x00, 0x06,
   0x30, 0x00, 0x00, 0x00, 0x00, 0x06, 0x30, 0xe0, 0xff, 0xff, 0x01, 0x07,
   0x70, 0xf0, 0xff, 0xff, 0x03, 0x03, 0x60, 0x00, 0x00, 0x00, 0x80, 0x03,
   0xe0, 0x00, 0x00, 0x00, 0xc0, 0x01, 0xc0, 0x01, 0x00, 0x00, 0xc0, 0x00,
   0x80, 0x03, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x07, 0x00, 0x00, 0x70, 0x00,
   0x00, 0x0e, 0x00, 0x00, 0x38, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x1e, 0x00,
   0x00, 0x78, 0x00, 0x80, 0x0f, 0x00, 0x00, 0xf0, 0x03, 0xe0, 0x03, 0x00,
   0x00, 0xc0, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x3f, 0x00, 0x00,
   0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const unsigned char outer_zone_rotate_bits[] PROGMEM= {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x0f, 0x00, 0x00,
   0x00, 0x00, 0xff, 0x7f, 0x00, 0x00, 0x00, 0xe0, 0x0f, 0xfc, 0x01, 0x00,
   0x00, 0xf0, 0x00, 0xc0, 0x07, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x0f, 0x00,
   0x00, 0x1e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x07, 0x00, 0x00, 0x38, 0x00,
   0x80, 0x03, 0x00, 0x00, 0x70, 0x00, 0xc0, 0x01, 0x00, 0x00, 0xe0, 0x00,
   0xc0, 0x00, 0x00, 0x00, 0xc0, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x80, 0x01,
   0x60, 0x10, 0x10, 0x01, 0x81, 0x03, 0x70, 0x38, 0xb8, 0x83, 0x03, 0x03,
   0x30, 0x78, 0xbc, 0xc7, 0x03, 0x07, 0x38, 0xf0, 0x1f, 0xff, 0x01, 0x06,
   0x18, 0xe0, 0x0f, 0xfe, 0x00, 0x06, 0x18, 0xc0, 0x07, 0x7c, 0x00, 0x0e,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x18, 0x00, 0x00, 0x00, 0x00, 0x0c,
   0x1c, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x0c,
   0x1c, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x0c,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x18, 0x00, 0x00, 0x00, 0x00, 0x0e,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x06, 0x38, 0x00, 0x00, 0x00, 0x00, 0x06,
   0x30, 0x00, 0x00, 0x00, 0x00, 0x06, 0x30, 0xe0, 0xff, 0xff, 0x01, 0x07,
   0x70, 0xf0, 0xff, 0xff, 0x03, 0x03, 0x60, 0x00, 0x00, 0x00, 0x80, 0x03,
   0xe0, 0x00, 0x00, 0x00, 0xc0, 0x01, 0xc0, 0x01, 0x00, 0x00, 0xc0, 0x00,
   0x80, 0x03, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x07, 0x00, 0x00, 0x70, 0x00,
   0x00, 0x0e, 0x00, 0x00, 0x38, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x1e, 0x00,
   0x00, 0x78, 0x00, 0x80, 0x0f, 0x00, 0x00, 0xf0, 0x03, 0xe0, 0x03, 0x00,
   0x00, 0xc0, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x3f, 0x00, 0x00,
   0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
   
static const unsigned char brew_acceptable_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x1f, 0x00, 0x00,
   0x00, 0xe0, 0xff, 0x7f, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf0, 0x01, 0x00,
   0x00, 0x3c, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x0f, 0x00,
   0x00, 0x07, 0x00, 0x00, 0x1e, 0x00, 0x80, 0x33, 0x00, 0x30, 0x18, 0x00,
   0xc0, 0x19, 0x00, 0xf0, 0x30, 0x00, 0xe0, 0x1c, 0x00, 0xe0, 0x73, 0x00,
   0x60, 0x0c, 0x00, 0x80, 0xef, 0x00, 0x30, 0x0e, 0x00, 0x00, 0xce, 0x00,
   0x38, 0xe6, 0x01, 0x70, 0xc0, 0x01, 0x18, 0xe3, 0x01, 0xf8, 0x80, 0x01,
   0x18, 0xf3, 0x03, 0xf8, 0x80, 0x01, 0x1c, 0xf0, 0x03, 0xf8, 0x80, 0x03,
   0x0c, 0xe0, 0x01, 0xf8, 0x00, 0x03, 0x0c, 0xe0, 0x01, 0x70, 0x00, 0x03,
   0x0c, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x03,
   0x0c, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x03,
   0x0c, 0x00, 0x00, 0x00, 0x08, 0x03, 0x0c, 0x00, 0x00, 0x00, 0x0c, 0x03,
   0x0c, 0x00, 0x00, 0x00, 0x0e, 0x03, 0x1c, 0x00, 0x00, 0x00, 0x0f, 0x03,
   0x18, 0x00, 0x00, 0x80, 0x87, 0x03, 0x18, 0x00, 0x00, 0xe0, 0x87, 0x01,
   0x38, 0x00, 0x00, 0xf0, 0x83, 0x01, 0x30, 0x60, 0x00, 0xfc, 0xc1, 0x00,
   0x70, 0xe0, 0x83, 0xff, 0xe0, 0x00, 0x60, 0xc0, 0xff, 0xff, 0x60, 0x00,
   0xe0, 0x80, 0xff, 0x3f, 0x70, 0x00, 0xc0, 0x01, 0xfe, 0x1f, 0x38, 0x00,
   0x80, 0x03, 0xf8, 0x07, 0x1c, 0x00, 0x00, 0x07, 0x00, 0x00, 0x0e, 0x00,
   0x00, 0x1e, 0x00, 0x80, 0x07, 0x00, 0x00, 0x7c, 0x00, 0xe0, 0x03, 0x00,
   0x00, 0xf0, 0x03, 0xfc, 0x00, 0x00, 0x00, 0xc0, 0xff, 0x3f, 0x00, 0x00,
   0x00, 0x00, 0xfe, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const unsigned char brew_acceptable_rotate_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x1f, 0x00, 0x00,
   0x00, 0xe0, 0xff, 0x7f, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf0, 0x01, 0x00,
   0x00, 0x3c, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x0f, 0x00,
   0x00, 0x07, 0x00, 0x00, 0x1e, 0x00, 0x80, 0xe3, 0x00, 0x30, 0x18, 0x00,
   0xc0, 0x79, 0x00, 0xf0, 0x30, 0x00, 0xe0, 0x3c, 0x00, 0xe0, 0x73, 0x00,
   0x60, 0x0c, 0x00, 0x80, 0xef, 0x00, 0x30, 0x0e, 0x00, 0x00, 0xce, 0x00,
   0x38, 0xe7, 0x01, 0x70, 0xc0, 0x01, 0x18, 0xe0, 0x01, 0xf8, 0x80, 0x01,
   0x18, 0xf0, 0x03, 0xf8, 0x80, 0x01, 0x1c, 0xf0, 0x03, 0xf8, 0x80, 0x03,
   0x0c, 0xe0, 0x01, 0xf8, 0x00, 0x03, 0x0c, 0xe0, 0x01, 0x70, 0x00, 0x03,
   0x0c, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x03,
   0x0c, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x03,
   0x0c, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x03,
   0x0c, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1c, 0x00, 0x00, 0x00, 0x0c, 0x03,
   0x18, 0x00, 0x00, 0x00, 0x87, 0x03, 0x18, 0x00, 0x00, 0x80, 0x87, 0x01,
   0x38, 0x00, 0x00, 0xe0, 0x83, 0x01, 0x30, 0x60, 0x00, 0xf8, 0xc1, 0x00,
   0x70, 0xe0, 0x83, 0xff, 0xe0, 0x00, 0x60, 0xc0, 0xff, 0xff, 0x60, 0x00,
   0xe0, 0x80, 0xff, 0x3f, 0x70, 0x00, 0xc0, 0x01, 0xfe, 0x1f, 0x38, 0x00,
   0x80, 0x03, 0xf8, 0x07, 0x1c, 0x00, 0x00, 0x07, 0x00, 0x00, 0x0e, 0x00,
   0x00, 0x1e, 0x00, 0x80, 0x07, 0x00, 0x00, 0x7c, 0x00, 0xe0, 0x03, 0x00,
   0x00, 0xf0, 0x03, 0xfc, 0x00, 0x00, 0x00, 0xc0, 0xff, 0x3f, 0x00, 0x00,
   0x00, 0x00, 0xfe, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const unsigned char brew_ready_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x07, 0x00, 0x00,
   0x00, 0x80, 0xff, 0x3f, 0x00, 0x00, 0x00, 0xe0, 0x07, 0xfc, 0x00, 0x00,
   0x00, 0xf8, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x3c, 0x00, 0x80, 0x07, 0x00,
   0x00, 0x0e, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x07, 0x00, 0x00, 0x1c, 0x00,
   0x80, 0x03, 0x00, 0x00, 0x38, 0x00, 0xc0, 0x01, 0x00, 0x00, 0x70, 0x00,
   0xe0, 0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0xc0, 0x00,
   0x70, 0x00, 0x07, 0x1c, 0xc0, 0x00, 0x30, 0x80, 0x0f, 0x3e, 0x80, 0x01,
   0x30, 0x80, 0x0f, 0x3e, 0x80, 0x01, 0x38, 0x80, 0x0f, 0x3e, 0x80, 0x03,
   0x18, 0x80, 0x0f, 0x3e, 0x00, 0x03, 0x18, 0x00, 0x07, 0x1c, 0x00, 0x03,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x03, 0x18, 0x00, 0x00, 0x00, 0x00, 0x03,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x03, 0x18, 0x00, 0x00, 0x00, 0x00, 0x03,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x03, 0x18, 0x00, 0x00, 0x00, 0x00, 0x03,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x03, 0x18, 0x04, 0x00, 0x00, 0x04, 0x03,
   0x38, 0x0c, 0x00, 0x00, 0x86, 0x03, 0x30, 0x1c, 0x00, 0x00, 0x87, 0x01,
   0x30, 0x18, 0x00, 0x80, 0x83, 0x01, 0x70, 0x78, 0x00, 0xc0, 0xc1, 0x00,
   0x60, 0xf0, 0x00, 0xe0, 0xe0, 0x00, 0xc0, 0xf8, 0x07, 0x7c, 0x60, 0x00,
   0xc0, 0x99, 0xff, 0x3f, 0x70, 0x00, 0x80, 0x9f, 0xff, 0x1f, 0x38, 0x00,
   0x00, 0x8f, 0xf9, 0x03, 0x1c, 0x00, 0x00, 0xce, 0x31, 0x00, 0x0e, 0x00,
   0x00, 0xc6, 0x38, 0x80, 0x07, 0x00, 0x00, 0xe7, 0x18, 0xe0, 0x01, 0x00,
   0x00, 0x67, 0x1c, 0xfe, 0x00, 0x00, 0x00, 0x67, 0xfc, 0x3f, 0x00, 0x00,
   0x00, 0x06, 0xfe, 0x03, 0x00, 0x00, 0x00, 0x0e, 0x07, 0x00, 0x00, 0x00,
   0x00, 0xfc, 0x03, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const unsigned char brew_ready_rotate_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x07, 0x00, 0x00,
   0x00, 0x80, 0xff, 0x3f, 0x00, 0x00, 0x00, 0xe0, 0x07, 0xfc, 0x00, 0x00,
   0x00, 0xf8, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x3c, 0x00, 0x80, 0x07, 0x00,
   0x00, 0x0e, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x07, 0x00, 0x00, 0x1c, 0x00,
   0x80, 0x03, 0x00, 0x00, 0x38, 0x00, 0xc0, 0x01, 0x00, 0x00, 0x70, 0x00,
   0xe0, 0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0xc0, 0x00,
   0x70, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x30, 0xc0, 0x07, 0xf8, 0x80, 0x01,
   0x30, 0xe0, 0x0f, 0xfc, 0x81, 0x01, 0x38, 0xf0, 0x1f, 0xfe, 0x83, 0x03,
   0x18, 0x78, 0x3c, 0x8f, 0x07, 0x03, 0x18, 0x38, 0x38, 0x07, 0x07, 0x03,
   0x18, 0x10, 0x10, 0x02, 0x02, 0x03, 0x18, 0x00, 0x00, 0x00, 0x00, 0x03,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x03, 0x18, 0x00, 0x00, 0x00, 0x00, 0x03,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x03, 0x18, 0x00, 0x00, 0x00, 0x00, 0x03,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x03, 0x18, 0x04, 0x00, 0x00, 0x04, 0x03,
   0x38, 0x0c, 0x00, 0x00, 0x86, 0x03, 0x30, 0x1c, 0x00, 0x00, 0x87, 0x01,
   0x30, 0x18, 0x00, 0x80, 0x83, 0x01, 0x70, 0x78, 0x00, 0xc0, 0xc1, 0x00,
   0x60, 0xf8, 0x00, 0xe0, 0xe0, 0x00, 0xc0, 0xb8, 0x07, 0x7c, 0x60, 0x00,
   0xc0, 0x19, 0xff, 0x3f, 0x70, 0x00, 0x80, 0x9f, 0xfb, 0x1f, 0x38, 0x00,
   0x00, 0x9f, 0xf1, 0x03, 0x1c, 0x00, 0x00, 0x9c, 0x31, 0x00, 0x0e, 0x00,
   0x00, 0x18, 0x38, 0x80, 0x07, 0x00, 0x00, 0x38, 0x1c, 0xe0, 0x01, 0x00,
   0x00, 0xf0, 0x0f, 0xfe, 0x00, 0x00, 0x00, 0xe0, 0xf7, 0x3f, 0x00, 0x00,
   0x00, 0x00, 0xfc, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
   
static const unsigned char coldstart_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0x1f, 0x00, 0x00,
   0x00, 0xc0, 0xff, 0xff, 0x00, 0x00, 0x00, 0xf0, 0x01, 0xe0, 0x03, 0x00,
   0x00, 0x78, 0x00, 0x80, 0x07, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x0e, 0x00,
   0x00, 0x0e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x07, 0x03, 0x38, 0x38, 0x00,
   0x80, 0xc3, 0x0f, 0x7c, 0x70, 0x00, 0xc0, 0x61, 0x18, 0x82, 0xe0, 0x00,
   0xc0, 0x70, 0x10, 0x81, 0xc1, 0x00, 0x60, 0x30, 0x30, 0x01, 0xc1, 0x01,
   0x70, 0x10, 0xb0, 0x01, 0x81, 0x01, 0x30, 0x10, 0xbc, 0xc1, 0x01, 0x03,
   0x30, 0x10, 0xbc, 0xc1, 0x01, 0x03, 0x38, 0x10, 0xbc, 0xc1, 0x01, 0x03,
   0x18, 0x30, 0x3c, 0xc1, 0x01, 0x06, 0x18, 0x20, 0x18, 0x83, 0x01, 0x06,
   0x18, 0x60, 0x0c, 0xc6, 0x00, 0x06, 0x18, 0xc0, 0x07, 0x7c, 0x00, 0x06,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x06, 0x18, 0x00, 0x00, 0x00, 0x00, 0x06,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x06, 0x18, 0x00, 0x00, 0x00, 0x00, 0x06,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x06, 0x38, 0x00, 0x1e, 0x00, 0x00, 0x07,
   0x30, 0x00, 0x3f, 0x00, 0x06, 0x03, 0x30, 0xc0, 0xf3, 0x80, 0x07, 0x03,
   0x70, 0xe0, 0xc0, 0xe7, 0x81, 0x03, 0x60, 0x70, 0x00, 0xff, 0x80, 0x01,
   0xe0, 0x30, 0x00, 0x3c, 0xc0, 0x01, 0xc0, 0x00, 0x00, 0x00, 0xe0, 0x00,
   0x80, 0x01, 0x00, 0x00, 0x60, 0x00, 0x80, 0x03, 0x00, 0x00, 0x70, 0x00,
   0x00, 0x07, 0x00, 0x00, 0x38, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x1e, 0x00,
   0x00, 0x3c, 0x00, 0x00, 0x0f, 0x00, 0x00, 0xf0, 0x00, 0xc0, 0x03, 0x00,
   0x00, 0xe0, 0x07, 0xfc, 0x01, 0x00, 0x00, 0x80, 0xff, 0x7f, 0x00, 0x00,
   0x00, 0x00, 0xf8, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const unsigned char coldstart_rotate_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0xff, 0x1f, 0x00, 0x00,
   0x00, 0xc0, 0xff, 0xff, 0x00, 0x00, 0x00, 0xf0, 0x01, 0xe0, 0x03, 0x00,
   0x00, 0x78, 0x00, 0x80, 0x07, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x0e, 0x00,
   0x00, 0x0e, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x07, 0x00, 0x00, 0x38, 0x00,
   0x80, 0x03, 0x00, 0x00, 0x70, 0x00, 0xc0, 0x01, 0x00, 0x00, 0xe0, 0x00,
   0xc0, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x60, 0x00, 0x00, 0x00, 0xc0, 0x01,
   0x70, 0x00, 0x00, 0x00, 0x80, 0x01, 0x30, 0x00, 0x00, 0x00, 0x00, 0x03,
   0x30, 0x10, 0x10, 0x01, 0x01, 0x03, 0x38, 0x38, 0xb8, 0x83, 0x03, 0x03,
   0x18, 0x78, 0xbc, 0xc7, 0x03, 0x06, 0x18, 0xf0, 0x1f, 0xff, 0x01, 0x06,
   0x18, 0xe0, 0x0f, 0xfe, 0x00, 0x06, 0x18, 0xc0, 0x07, 0x7c, 0x00, 0x06,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x06, 0x18, 0x00, 0x00, 0x00, 0x00, 0x06,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x06, 0x18, 0x00, 0x00, 0x00, 0x00, 0x06,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x06, 0x38, 0x00, 0x1e, 0x00, 0x00, 0x07,
   0x30, 0x00, 0x3f, 0x00, 0x06, 0x03, 0x30, 0xc0, 0xf3, 0x80, 0x07, 0x03,
   0x70, 0xe0, 0xc0, 0xe7, 0x81, 0x03, 0x60, 0x70, 0x00, 0xff, 0x80, 0x01,
   0xe0, 0x30, 0x00, 0x3c, 0xc0, 0x01, 0xc0, 0x00, 0x00, 0x00, 0xe0, 0x00,
   0x80, 0x01, 0x00, 0x00, 0x60, 0x00, 0x80, 0x03, 0x00, 0x00, 0x70, 0x00,
   0x00, 0x07, 0x00, 0x00, 0x38, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x1e, 0x00,
   0x00, 0x3c, 0x00, 0x00, 0x0f, 0x00, 0x00, 0xf0, 0x00, 0xc0, 0x03, 0x00,
   0x00, 0xe0, 0x07, 0xfc, 0x01, 0x00, 0x00, 0x80, 0xff, 0x7f, 0x00, 0x00,
   0x00, 0x00, 0xf8, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const unsigned char brewing_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x07, 0x00, 0x00,
   0x00, 0xc0, 0xff, 0x3f, 0x00, 0x00, 0x00, 0xf0, 0x03, 0xfc, 0x00, 0x00,
   0x00, 0x7c, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x1e, 0x00, 0x80, 0x07, 0x00,
   0x00, 0x07, 0x00, 0x00, 0x0e, 0x00, 0x80, 0x03, 0x00, 0x00, 0x1c, 0x00,
   0xc0, 0x01, 0x00, 0x00, 0x38, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x70, 0x00,
   0x70, 0x00, 0x00, 0x00, 0x60, 0x00, 0x30, 0xc0, 0x00, 0x18, 0xe0, 0x00,
   0x38, 0xe0, 0x01, 0x3c, 0xc0, 0x00, 0x18, 0xf0, 0x03, 0x7e, 0xc0, 0x01,
   0x1c, 0xf0, 0x03, 0x7e, 0x80, 0x01, 0x0c, 0xf0, 0x03, 0x7e, 0x80, 0x03,
   0x0c, 0xe0, 0x01, 0x3c, 0x00, 0x03, 0x0e, 0xc0, 0x00, 0x18, 0x00, 0x03,
   0x06, 0x00, 0x00, 0x00, 0x00, 0x03, 0x06, 0x00, 0x00, 0x00, 0x00, 0x03,
   0x06, 0x00, 0x00, 0x00, 0x00, 0x07, 0x06, 0x00, 0x00, 0x00, 0x00, 0x07,
   0xc6, 0x00, 0x00, 0x00, 0x10, 0x07, 0xc6, 0x03, 0x00, 0x00, 0x1e, 0x07,
   0xc6, 0x0f, 0x00, 0x80, 0x1f, 0x03, 0xce, 0x7f, 0x00, 0xe0, 0x1f, 0x03,
   0x8c, 0xff, 0x03, 0xfc, 0x1f, 0x03, 0x8c, 0xff, 0xff, 0xff, 0x1f, 0x03,
   0x8c, 0xff, 0xff, 0xff, 0x8f, 0x03, 0x1c, 0xff, 0xff, 0xff, 0x8f, 0x01,
   0x18, 0xfe, 0xff, 0xff, 0xc7, 0x01, 0x38, 0xfe, 0xff, 0xff, 0xc3, 0x00,
   0x70, 0xfc, 0xff, 0xff, 0xe1, 0x00, 0x60, 0xf8, 0xff, 0xff, 0x71, 0x00,
   0xe0, 0xf0, 0xff, 0x7f, 0x38, 0x00, 0xc0, 0xc1, 0xff, 0x3f, 0x1c, 0x00,
   0x80, 0x03, 0xff, 0x0f, 0x1e, 0x00, 0x00, 0x0f, 0xf8, 0x00, 0x0f, 0x00,
   0x00, 0x1e, 0x00, 0xc0, 0x03, 0x00, 0x00, 0xf8, 0x00, 0xf0, 0x01, 0x00,
   0x00, 0xe0, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x80, 0xff, 0x1f, 0x00, 0x00,
   0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const unsigned char brewing_rotate_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x07, 0x00, 0x00,
   0x00, 0xc0, 0xff, 0x3f, 0x00, 0x00, 0x00, 0xf0, 0x03, 0xfc, 0x00, 0x00,
   0x00, 0x7c, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x1e, 0x00, 0x80, 0x07, 0x00,
   0x00, 0x07, 0x00, 0x00, 0x0e, 0x00, 0x80, 0x03, 0x00, 0x00, 0x1c, 0x00,
   0xc0, 0x01, 0x00, 0x00, 0x38, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x70, 0x00,
   0x70, 0x00, 0x00, 0x00, 0x60, 0x00, 0x30, 0xe0, 0x03, 0x7c, 0xe0, 0x00,
   0x38, 0xf0, 0x07, 0xfe, 0xc0, 0x00, 0x18, 0xf8, 0x0f, 0xff, 0xc1, 0x01,
   0x1c, 0x3c, 0x9e, 0xc7, 0x83, 0x01, 0x0c, 0x1c, 0x9c, 0x83, 0x83, 0x03,
   0x0c, 0x08, 0x08, 0x01, 0x01, 0x03, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x03,
   0x06, 0x00, 0x00, 0x00, 0x00, 0x03, 0x06, 0x00, 0x00, 0x00, 0x00, 0x03,
   0x06, 0x00, 0x00, 0x00, 0x00, 0x07, 0x06, 0x00, 0x00, 0x00, 0x00, 0x07,
   0xc6, 0x00, 0x00, 0x00, 0x10, 0x07, 0xc6, 0x03, 0x00, 0x00, 0x1e, 0x07,
   0xc6, 0x0f, 0x00, 0x80, 0x1f, 0x03, 0xce, 0x7f, 0x00, 0xe0, 0x1f, 0x03,
   0x8c, 0xff, 0x03, 0xfc, 0x1f, 0x03, 0x8c, 0xff, 0xff, 0xff, 0x1f, 0x03,
   0x8c, 0xff, 0xff, 0xff, 0x8f, 0x03, 0x1c, 0xff, 0xff, 0xff, 0x8f, 0x01,
   0x18, 0xfe, 0xff, 0xff, 0xc7, 0x01, 0x38, 0xfe, 0xff, 0xff, 0xc3, 0x00,
   0x70, 0xfc, 0xff, 0xff, 0xe1, 0x00, 0x60, 0xf8, 0xff, 0xff, 0x71, 0x00,
   0xe0, 0xf0, 0xff, 0x7f, 0x38, 0x00, 0xc0, 0xc1, 0xff, 0x3f, 0x1c, 0x00,
   0x80, 0x03, 0xff, 0x0f, 0x1e, 0x00, 0x00, 0x0f, 0xf8, 0x00, 0x0f, 0x00,
   0x00, 0x1e, 0x00, 0xc0, 0x03, 0x00, 0x00, 0xf8, 0x00, 0xf0, 0x01, 0x00,
   0x00, 0xe0, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x80, 0xff, 0x1f, 0x00, 0x00,
   0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/*
- ["washing hand"](https://thenounproject.com/matfine/collection/cleaning-icon/?i=2749704) by [Mat fine](https://thenounproject.com/matfine) from [the Noun Project](https://thenounproject.com/matfine/collection/cleaning-icon/?i=2749704) licensed under [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/legalcode)
*/
static const unsigned char clean_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00,
   0x00, 0x38, 0x00, 0x00, 0x36, 0x00, 0x00, 0x44, 0x00, 0x00, 0x14, 0x00,
   0x00, 0x44, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x6c, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x38, 0xfc, 0xe1, 0x01, 0x00, 0x00, 0x00, 0x06, 0x1b, 0x06, 0x00,
   0x00, 0x00, 0x03, 0x0e, 0x04, 0x00, 0x00, 0xbc, 0x39, 0x0c, 0x0c, 0x00,
   0x00, 0xc3, 0x04, 0x01, 0x0c, 0x00, 0x00, 0x81, 0x04, 0x01, 0x04, 0x00,
   0x80, 0x01, 0x00, 0x00, 0x04, 0x00, 0x80, 0xc0, 0x03, 0x00, 0x06, 0x00,
   0x80, 0x61, 0x02, 0x07, 0x0c, 0x00, 0x00, 0x21, 0x82, 0x09, 0x10, 0x00,
   0x00, 0x33, 0x62, 0x28, 0x10, 0x00, 0x80, 0x11, 0x33, 0xf8, 0x10, 0x00,
   0x80, 0x08, 0x0d, 0x8c, 0x10, 0x00, 0x80, 0x88, 0x07, 0x83, 0x18, 0x00,
   0x80, 0x84, 0xc1, 0xc0, 0x0c, 0x00, 0x80, 0x06, 0x60, 0xf0, 0x03, 0x00,
   0x00, 0x03, 0x18, 0x18, 0x01, 0x00, 0x00, 0x03, 0x0c, 0x06, 0xc1, 0x00,
   0x00, 0x01, 0x00, 0x83, 0xb1, 0x01, 0x80, 0x00, 0xc0, 0xc0, 0x1c, 0x01,
   0x80, 0x00, 0x60, 0xf0, 0x06, 0x03, 0x80, 0x00, 0x00, 0x98, 0x06, 0x01,
   0x60, 0x00, 0x00, 0x86, 0x8c, 0x00, 0x30, 0x00, 0x80, 0x61, 0x88, 0x00,
   0x0c, 0x00, 0xc0, 0x18, 0x70, 0x00, 0x04, 0x00, 0x00, 0x0c, 0x00, 0x00,
   0x06, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x80, 0x01, 0x00, 0x00,
   0x0c, 0x00, 0x60, 0x00, 0x00, 0x00, 0x08, 0x00, 0x30, 0x00, 0x00, 0x00,
   0x10, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x30, 0xf0, 0x03, 0x00, 0x00, 0x00,
   0x20, 0x8c, 0x00, 0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00, 0x00,
   0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/*
- ["washing hand"](https://thenounproject.com/matfine/collection/cleaning-icon/?i=2749704) by [Mat fine](https://thenounproject.com/matfine) from [the Noun Project](https://thenounproject.com/matfine/collection/cleaning-icon/?i=2749704) licensed under [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/legalcode)
*/
static const unsigned char clean_rotate_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00,
   0x00, 0xe0, 0x00, 0x00, 0xd8, 0x00, 0x00, 0x10, 0x01, 0x00, 0x50, 0x00,
   0x00, 0x10, 0x01, 0x00, 0x70, 0x00, 0x00, 0xb0, 0x01, 0x00, 0x00, 0x00,
   0x00, 0xe0, 0xf0, 0x87, 0x07, 0x00, 0x00, 0x00, 0x18, 0x6c, 0x18, 0x00,
   0x00, 0x00, 0x0c, 0x38, 0x10, 0x00, 0x00, 0xf0, 0xe6, 0x30, 0x30, 0x00,
   0x00, 0x0c, 0x13, 0x04, 0x30, 0x00, 0x00, 0x04, 0x12, 0x04, 0x10, 0x00,
   0x00, 0x06, 0x00, 0x00, 0x10, 0x00, 0x00, 0x02, 0x0f, 0x00, 0x18, 0x00,
   0x00, 0x86, 0x09, 0x1c, 0x30, 0x00, 0x00, 0x84, 0x08, 0x26, 0x40, 0x00,
   0x00, 0xcc, 0x88, 0xa1, 0x40, 0x00, 0x00, 0x46, 0xcc, 0xe0, 0x43, 0x00,
   0x00, 0x22, 0x34, 0x30, 0x42, 0x00, 0x00, 0x22, 0x1e, 0x0c, 0x62, 0x00,
   0x00, 0x12, 0x06, 0x03, 0x33, 0x00, 0x00, 0x1a, 0x80, 0xc1, 0x0f, 0x00,
   0x00, 0x0c, 0x60, 0x60, 0x04, 0x00, 0x00, 0x0c, 0x30, 0x18, 0x04, 0x03,
   0x00, 0x04, 0x00, 0x0c, 0xc6, 0x06, 0x00, 0x02, 0x00, 0x03, 0x73, 0x04,
   0x00, 0x02, 0x80, 0xc1, 0x1b, 0x0c, 0x00, 0x02, 0x00, 0x60, 0x1a, 0x04,
   0x80, 0x01, 0x00, 0x18, 0x32, 0x02, 0xc0, 0x00, 0x00, 0x86, 0x21, 0x02,
   0x30, 0x00, 0x00, 0x63, 0xc0, 0x01, 0x10, 0x00, 0x00, 0x30, 0x00, 0x00,
   0x18, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x10, 0x00, 0x00, 0x06, 0x00, 0x00,
   0x30, 0x00, 0x80, 0x01, 0x00, 0x00, 0x20, 0x00, 0xc0, 0x00, 0x00, 0x00,
   0x40, 0x00, 0x30, 0x00, 0x00, 0x00, 0xc0, 0xc0, 0x0f, 0x00, 0x00, 0x00,
   0x80, 0x30, 0x02, 0x00, 0x00, 0x00, 0x80, 0x19, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/*
- ["menu"](https://thenounproject.com/icon/menu-2943760/) by [Mat fine](https://thenounproject.com/matfine) from [the Noun Project](https://thenounproject.com/icon/tasks-2943947/) licensed under [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/legalcode)
*/
static const unsigned char menu_bits[] PROGMEM = {
   0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x21, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xe1, 0x00, 0x00, 0x00,
   0x00, 0xe0, 0xe1, 0x01, 0x00, 0x00, 0xc0, 0x1f, 0x00, 0xff, 0x00, 0x00,
   0x30, 0x30, 0x00, 0x81, 0x03, 0x00, 0x18, 0xe0, 0xff, 0x00, 0x02, 0x00,
   0x08, 0x00, 0x00, 0x00, 0x06, 0x00, 0x08, 0x00, 0x00, 0x00, 0x04, 0x00,
   0x08, 0x00, 0x00, 0x00, 0x04, 0x00, 0x08, 0x20, 0x00, 0x00, 0x04, 0x00,
   0x08, 0x7c, 0x00, 0x00, 0x84, 0x07, 0x08, 0xf6, 0x00, 0x00, 0x84, 0x08,
   0x08, 0xdb, 0xfc, 0x3f, 0x84, 0x08, 0x08, 0x49, 0x00, 0x00, 0x84, 0x08,
   0x08, 0x27, 0x00, 0x00, 0x84, 0x08, 0x08, 0x1f, 0x00, 0x00, 0x84, 0x08,
   0x08, 0x00, 0x00, 0x00, 0x84, 0x08, 0x08, 0x00, 0x00, 0x00, 0x84, 0x08,
   0x08, 0x00, 0x00, 0x00, 0x84, 0x08, 0x08, 0x20, 0x00, 0x00, 0x84, 0x08,
   0x08, 0x7c, 0x00, 0x00, 0x84, 0x08, 0x08, 0xb6, 0x00, 0x00, 0x84, 0x08,
   0x08, 0x5b, 0xfc, 0x3f, 0x84, 0x08, 0x08, 0x4d, 0x00, 0x00, 0x84, 0x08,
   0x08, 0x37, 0x00, 0x00, 0x84, 0x08, 0x08, 0x1e, 0x00, 0x00, 0x84, 0x08,
   0x08, 0x00, 0x00, 0x00, 0x84, 0x08, 0x08, 0x00, 0x00, 0x00, 0x84, 0x08,
   0x08, 0x00, 0x00, 0x00, 0x84, 0x08, 0x08, 0x70, 0x00, 0x00, 0x84, 0x08,
   0x08, 0xfc, 0x00, 0x00, 0x84, 0x08, 0x08, 0x96, 0x00, 0x00, 0x84, 0x0f,
   0x08, 0x5b, 0xfc, 0x3f, 0x84, 0x0d, 0x08, 0x6d, 0x00, 0x00, 0x04, 0x07,
   0x08, 0x3f, 0x00, 0x00, 0x04, 0x03, 0x08, 0x1e, 0x00, 0x00, 0x04, 0x00,
   0x08, 0x00, 0x00, 0x00, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x04, 0x00,
   0x08, 0x00, 0x00, 0x00, 0x06, 0x00, 0x18, 0x00, 0x00, 0x00, 0x02, 0x00,
   0x30, 0x00, 0x00, 0x00, 0x03, 0x00, 0xe0, 0xff, 0xff, 0xff, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/*
- ["menu"](https://thenounproject.com/icon/menu-2943760/) by [Mat fine](https://thenounproject.com/matfine) from [the Noun Project](https://thenounproject.com/icon/tasks-2943947/) licensed under [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/legalcode)
*/
static const unsigned char menu_rotate_bits[] PROGMEM = {
   0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x21, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xe1, 0x00, 0x00, 0x00,
   0x00, 0xe0, 0xe1, 0x01, 0x00, 0x00, 0xc0, 0x1f, 0x00, 0xff, 0x00, 0x00,
   0x30, 0x30, 0x00, 0x81, 0x03, 0x00, 0x18, 0xe0, 0xff, 0x00, 0x02, 0x00,
   0x08, 0x00, 0x00, 0x00, 0x06, 0x00, 0x08, 0x00, 0x00, 0x00, 0x04, 0x00,
   0x08, 0x00, 0x00, 0x00, 0x04, 0x00, 0x08, 0x20, 0x00, 0x00, 0x04, 0x00,
   0x08, 0x7c, 0x00, 0x00, 0x84, 0x07, 0x08, 0xf6, 0x00, 0x00, 0x84, 0x08,
   0x08, 0xdb, 0xfc, 0x3f, 0x84, 0x08, 0x08, 0x49, 0x00, 0x00, 0x84, 0x08,
   0x08, 0x27, 0x00, 0x00, 0x84, 0x08, 0x08, 0x1f, 0x00, 0x00, 0x84, 0x08,
   0x08, 0x00, 0x00, 0x00, 0x84, 0x08, 0x08, 0x00, 0x00, 0x00, 0x84, 0x08,
   0x08, 0x00, 0x00, 0x00, 0x84, 0x08, 0x08, 0x20, 0x00, 0x00, 0x84, 0x08,
   0x08, 0x7c, 0x00, 0x00, 0x84, 0x08, 0x08, 0xb6, 0x00, 0x00, 0x84, 0x08,
   0x08, 0x5b, 0xfc, 0x3f, 0x84, 0x08, 0x08, 0x4d, 0x00, 0x00, 0x84, 0x08,
   0x08, 0x37, 0x00, 0x00, 0x84, 0x08, 0x08, 0x1e, 0x00, 0x00, 0x84, 0x08,
   0x08, 0x00, 0x00, 0x00, 0x84, 0x08, 0x08, 0x00, 0x00, 0x00, 0x84, 0x08,
   0x08, 0x00, 0x00, 0x00, 0x84, 0x08, 0x08, 0x70, 0x00, 0x00, 0x84, 0x08,
   0x08, 0xfc, 0x00, 0x00, 0x84, 0x08, 0x08, 0x96, 0x00, 0x00, 0x84, 0x0f,
   0x08, 0x5b, 0xfc, 0x3f, 0x84, 0x0d, 0x08, 0x6d, 0x00, 0x00, 0x04, 0x07,
   0x08, 0x3f, 0x00, 0x00, 0x04, 0x03, 0x08, 0x1e, 0x00, 0x00, 0x04, 0x00,
   0x08, 0x00, 0x00, 0x00, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x04, 0x00,
   0x08, 0x00, 0x00, 0x00, 0x06, 0x00, 0x18, 0x00, 0x00, 0x00, 0x02, 0x00,
   0x30, 0x00, 0x00, 0x00, 0x03, 0x00, 0xe0, 0xff, 0xff, 0xff, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
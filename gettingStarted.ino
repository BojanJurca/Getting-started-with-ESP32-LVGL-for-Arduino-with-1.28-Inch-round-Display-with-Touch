/*

    A working example combined from many different sources, like:

    http://pan.jczn1688.com/directlink/1/ESP32%20module/1.28inch_ESP32-2424S012.zip

    https://docs.lvgl.io/master/integration/framework/arduino.html#set-up-drivers

    https://docs.lvgl.io/master/get-started/quick-overview.html

    https://www.elecrow.com/wiki/index.php?title=LVGL_ESP32_Display_Tutorial-A_Step-by-Step_Guide_to_LVGL_GUI_Development


*/


#include <lvgl.h>
#include <LovyanGFX.hpp>

// Backlight GPIO (these definitions are of no use since setBrightness is kinfa not working)
#define TFT_BL 3 
#define GFX_BL 3

// Renewal rate of screen division
static const uint32_t screenWidth = 240;
static const uint32_t screenHeight = 240;

// #define off_pin 35
#define BUF_SIZE 120

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf [2][screenWidth * BUF_SIZE];


class LGFX : public lgfx::LGFX_Device {
    
    private:
        
        lgfx::Panel_GC9A01 _panel_instance;
        lgfx::Bus_SPI _bus_instance;

    public:
      
        LGFX (void) {
            {   // Configuring the SPI bus
                auto cfg = _bus_instance.config ();

                cfg.spi_host = SPI2_HOST; // Select the SPI to use ESP32-S2,C3: SPI2_HOST or SPI3_HOST / ESP32: VSPI_HOST or HSPI_HOST
                // * Due to the ESP-IDF version update, the description of VSPI_HOST and HSPI_HOST will be deprecated, so if an error occurs, please use SPI2_HOST and SPI3_HOST instead
                cfg.spi_mode = 0; // Set SPI communication mode (0 ~ 3)
                cfg.freq_write = 80000000; // SPI time (up to 80MHz, four or five inputs divided by 80MHz to get an integer)
                cfg.freq_read = 20000000; // SPI time when connected
                cfg.spi_3wire = true; // Set true if receiving is done via MOSI pin
                cfg.use_lock = true; // Usage lock time setting true
                cfg.dma_channel = SPI_DMA_CH_AUTO; // Set the DMA channel to use (0=DMA not used / 1=1ch / 2=ch / SPI_DMA_CH_AUTO=automatic setting)
                // * Due to the ESP-IDF version upgrade, SPI_DMA_CH_AUTO (automatic setting) is recommended for the DMA channel. Specifying 1ch or 2ch is not recommended.
                cfg.pin_sclk = 6; // Set SPI SCLK pin number
                cfg.pin_mosi = 7; // Set SPI CLK pin number
                cfg.pin_miso = -1; // Set SPI MISO pin number (-1 = disable)
                cfg.pin_dc = 2; // Set SPI D/C pin number (-1 = disable)

                _bus_instance.config (cfg); // Apply the settings to the bus.
                _panel_instance.setBus (&_bus_instance); // Set the bus to the panel.    
            }
            {   // Set display panel control.
                auto cfg = _panel_instance.config (); // Get the structure for display panel settings.

                cfg.pin_cs = 10; // Pin number to which CS is connected (-1 = disable)
                cfg.pin_rst = -1; // Pin number to which RST is connected (-1 = disable)
                cfg.pin_busy = -1; // Pin number to which BUSY is connected (-1 = disable)

                // * The following setting values ​​are general initial values ​​set for each panel and the pin number to which BUSY is connected (-1 = disable), so please comment out any unknown items and try again.
                cfg.memory_width = 240; // Maximum width supported by the driver IC
                cfg.memory_height = 240; // Maximum height supported by the driver IC
                cfg.panel_width = 240; // Actual displayable width
                cfg.panel_height = 240; // Actual display height
                cfg.offset_x = 0; // Panel X direction offset amount
                cfg.offset_y = 0; // Panel Y direction offset amount
                cfg.offset_rotation = 0; // directional rotational shift 0~7 (4~7 is inverted)
                cfg.dummy_read_pixel = 8; // Number of images that can be read before reading
                cfg.dummy_read_bits = 1; // imaginary reading order of numbers other than reading image elements
                cfg.readable = false; // As long as the number of acquisitions is as high as possible, the setting is true
                cfg.invert = true; // As a result, the brightness and darkness of the board is reversed, and the setting is true
                cfg.rgb_order = false; // As a result, the red color and Japanese blue color are replaced, and the setting is true
                cfg.dlen_16bit = false; // From 16th position to 16th position, the length of the number of transfers is set as true.
                cfg.bus_shared = false; // As a result, the SD card can be shared, the setting is true (use drawJpgFile etc.)
                _panel_instance.config (cfg);
              }

              setPanel (&_panel_instance); // Set the panel to use.
        }
};

// Create an instance of the prepared class.
LGFX tft;


#include <Wire.h>

class CST816D {

    public:  

        #define I2C_ADDR_CST816D 0x15

        // click
        enum GESTURE {
            None = 0x00, // no gesture
            SlideDown = 0x01, // slide down
            SlideUp = 0x02, // slide up
            SlideLeft = 0x03, // slide left
            SlideRight = 0x04, // slide to the right
            SingleTap = 0x05, // click
            DoubleTap = 0x0B, // double click
            LongPress = 0x0C // long press
        };    
      
        CST816D () {
            _sda = 4; // I2C_SDA GPIO
            _scl = 5; // I2C_SCL GPIO
            _rst = 1; // TP_RST GPIO
            _int = 0; // TP_INT GPIO
        }

        void begin (void) {
            // Initialize I2C
            if (_sda != -1 && _scl != -1) {
                Wire.begin (_sda, _scl);
            } else {
                Wire.begin ();
            }

            // Int Pin Configuration
            if (_int != -1) {
                pinMode (_int, OUTPUT);
                digitalWrite (_int, HIGH); // High level
                delay (1);
                digitalWrite (_int, LOW); // Low level
                delay (1);
            }

            // Reset Pin Configuration
            if (_rst != -1) {
                pinMode (_rst, OUTPUT);
                digitalWrite (_rst, LOW);
                delay (10);
                digitalWrite (_rst, HIGH);
                delay (300);
            }

            // Initialize Touch
            i2c_write (0xFE, 0XFF); // Do not automatically enter low power mode.
        }

        bool getTouch (uint16_t *x, uint16_t *y, uint8_t *gesture) {
            bool FingerIndex = false;
            FingerIndex = (bool) i2c_read (0x02);

            *gesture = i2c_read (0x01);
            if (!(*gesture == SlideUp || *gesture == SlideDown)) {
                *gesture = None;
            }

            uint8_t data [4];
            i2c_read_continuous (0x03,data,4);
            *x = ((data [0] & 0x0f) << 8) | data [1];
            *y = ((data [2] & 0x0f) << 8) | data [3];

            // *x=240-*x;

            return FingerIndex;
        }

    private:
        
        int8_t _sda, _scl, _rst, _int;

        uint8_t i2c_read (uint8_t addr) {
            uint8_t rdData;
            uint8_t rdDataCount;
            do {
                Wire.beginTransmission (I2C_ADDR_CST816D);
                Wire.write (addr);
                Wire.endTransmission (false); // Restart
                rdDataCount = Wire.requestFrom (I2C_ADDR_CST816D, 1);
            } while (rdDataCount == 0);
            while (Wire.available ()) {
                rdData = Wire.read ();
            }
            return rdData;
        }

        uint8_t i2c_read_continuous (uint8_t addr, uint8_t *data, uint32_t length) {
            Wire.beginTransmission (I2C_ADDR_CST816D);
            Wire.write (addr);
            if (Wire.endTransmission (true)) return -1;
            Wire.requestFrom (I2C_ADDR_CST816D, length);
            for (int i = 0; i < length; i++) {
                *data++ = Wire.read ();
            }
            return 0;
        }

        void i2c_write (uint8_t addr, uint8_t data) {
            Wire.beginTransmission (I2C_ADDR_CST816D);
            Wire.write (addr);
            Wire.write (data);
            Wire.endTransmission ();
        }

        uint8_t i2c_write_continuous (uint8_t addr, const uint8_t *data, uint32_t length) {
          Wire.beginTransmission (I2C_ADDR_CST816D);
          Wire.write (addr);
          for (int i = 0; i < length; i++) {
            Wire.write (*data++);
          }
          if (Wire.endTransmission (true)) return -1;
          return 0;
        }

};

// Create an instance of the prepared class.
CST816D touch;


// User interface graphic elements.
lv_obj_t *ui_Screen1; // the first and only screen in this example
lv_obj_t *ui_Label1; // the first and only label in this example
lv_obj_t *ui_Button1; // the first and only button in this example
lv_obj_t *ui_Button1_Label; 


// Display flushing 
void my_disp_flush (lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    if (tft.getStartCount () == 0)
        tft.endWrite ();

    tft.pushImageDMA (area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1, (lgfx::swap565_t *) &color_p->full);

    lv_disp_flush_ready (disp); // tell lvgl that flushing is done 
}


static void event_handler (lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code (e);

    if(code == LV_EVENT_CLICKED) {
        Serial.println ("Button clicked");

        lv_label_set_text (ui_Label1, "Clicked!");
        lv_obj_align (ui_Label1, LV_ALIGN_CENTER, 0, 0);
    }
}


// Read the touchpad
void my_touchpad_read (lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
    bool touched;
    uint8_t gesture;
    uint16_t touchX, touchY;

    touched = touch.getTouch(&touchX, &touchY, &gesture);
    if (!touched) {
        data->state = LV_INDEV_STATE_REL;
    } else {
        data->state = LV_INDEV_STATE_PR;

        // Set the coordinates
        data->point.x = touchX;
        data->point.y = touchY;
    }
}


void setup () { // https://lovyangfx.readthedocs.io/en/latest/02_using.html

    delay (1000); // this will give you some time to press "upload" button before uploading a new sketch in case a faulty sketch is already uploaded, which may result in: Failed uploading: uploading error: exit status 2
    
    Serial.begin (115200); // prepare for possible serial debug          

    // GFX initialization
    Serial.println ("GFX initialization");

    tft.init ();
    tft.initDMA ();


    // Turn on the backlight (GPIO 3)
    tft.fillScreen (TFT_BLACK);
    // tft.setBrightness (128); // 0-255
    pinMode (GFX_BL, OUTPUT); 
    digitalWrite (GFX_BL, HIGH); 


        // Test GFX
        Serial.println ("Test GFX");

        tft.drawCircle(120, 120, 119, TFT_GOLD);
        tft.setCursor (120, 120); 
        tft.setTextSize	(1);
        tft.setTextColor (TFT_GOLD); 
        tft.println ("GFX"); 

    delay (3000);


    // LVGL (and display bufefr) initialization
    Serial.println ("LVGL initialization");

    lv_init ();
    lv_disp_draw_buf_init (&draw_buf, buf [0], buf [1], screenWidth * BUF_SIZE); 

    // Initialize the display
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init (&disp_drv);
    // Change the following line to your display resolution
    disp_drv.hor_res = screenWidth; // 240
    disp_drv.ver_res = screenHeight; // 240
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register (&disp_drv);


        // Test LVGL
        //    my_disp_flush function is needed
        //    lv_timer_handler () or lv_task_handler () functions need to be celled in loop ()
        //    event_handler function is needed to handle events, like putton pressed

        Serial.println ("Test LVGL");

            // Create Screen1 first
            ui_Screen1 = lv_obj_create (NULL);
            lv_obj_clear_flag (ui_Screen1, LV_OBJ_FLAG_SCROLLABLE); // make it non-scrollable

            // Create label1 on Screen1: https://docs.arduino.cc/tutorials/portenta-h7/creating-gui-with-lvgl/
            ui_Label1 = lv_label_create (ui_Screen1);
            lv_obj_set_width (ui_Label1, LV_SIZE_CONTENT);  // 1
            lv_obj_set_height (ui_Label1, LV_SIZE_CONTENT); // 1
            lv_obj_set_x (ui_Label1, 100);
            lv_obj_set_y (ui_Label1, 0);
            lv_label_set_text (ui_Label1, "LVGL label");
            lv_obj_align (ui_Label1, LV_ALIGN_CENTER, 0, 0); // center the Label1 on Screen1

            // Load Screen1
            // lv_scr_load (ui_Screen1);
            lv_scr_load_anim (ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, false); // Screen1 will actually be loaded later in loop function that calls lv_timer_handler ()

            // Create button1 on active screen, which is, again, Screen1
            // ui_Button1 = lv_btn_create (lv_scr_act ());
            ui_Button1 = lv_btn_create (ui_Screen1);
            lv_obj_add_event_cb (ui_Button1, event_handler, LV_EVENT_ALL, NULL);
            lv_obj_align (ui_Button1, LV_ALIGN_CENTER, 0, 40);
            lv_obj_clear_flag (ui_Button1, LV_OBJ_FLAG_PRESS_LOCK);

            ui_Button1_Label = lv_label_create (ui_Button1);
            lv_label_set_text (ui_Button1_Label, "LVGL button");
            lv_obj_center (ui_Button1_Label);

        // Test touchpad
        //    my_touchpad_read function is needed

        Serial.println ("Test touchpad");

            touch.begin ();

            // Initialize the (dummy) input device driver
            {
                static lv_indev_drv_t indev_drv;
                lv_indev_drv_init (&indev_drv);
                indev_drv.type = LV_INDEV_TYPE_POINTER;
                indev_drv.read_cb = my_touchpad_read;
                lv_indev_drv_register (&indev_drv);
            }
}

void loop () {
  lv_timer_handler (); // let the GUI do its work
  delay (5);
}

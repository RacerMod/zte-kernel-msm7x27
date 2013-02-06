struct usb_function_map usb_functions_map[] = {
	{"diag", 0},
	{"modem", 1},
	{"nmea", 2},
	{"mass_storage", 3},
	{"adb", 4},
        //{"ethernet", 5},
};

/* dynamic composition */
struct usb_composition usb_func_composition[] = {
	
	{
		.product_id     = 0x0112,
		.functions	    = 0x01, /* 000001 */
	},

	{
		.product_id     = 0x0111,
		.functions	    = 0x07, /* 000111 */
	},
	
	{
		.product_id     = 0x1355, /*ZTE_USBCONFIG_001*/
		.functions	    = 0x0A, /* 001010 */
	},

	{
		.product_id     = 0x1354, /*ZTE_USBCONFIG_001*/
		.functions	    = 0x1A, /* 011010 */
	},

	{
		.product_id     = 0x1353,
		.functions	    = 0x08, /* 001000: ms */
	},
	
	{
		.product_id     = 0x0083,
		.functions	    = 0x08, /* 001000: ms +cdrom*/
	},

	{
		.product_id     = 0x1352,
		.functions	    = 0x10, /* 010000 */
	},

	{
		.product_id     = 0x1351,
		.functions	    = 0x18, /* 011000 */
	},

	{
		.product_id     = 0x1350,
		.functions	    = 0x1F, /* 011111 */
	},

};
#endif
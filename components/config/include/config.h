
typedef struct _wifi_cnf {
	char ssid[32];
	char password[64];
} wifi_cnf;

typedef struct _sender_cnf {
	char url[256];
	uint interval;
} sender_cnf;

void get_wifi_conf(wifi_cnf *cnf);
void set_wifi_conf(wifi_cnf *cnf);

void get_sender_conf(sender_cnf *cnf);
void set_sender_conf(sender_cnf *cnf);

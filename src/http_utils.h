/*
 *  HTTP functions to connect to our Heroku API
 */

#ifndef __AWS_H__
#define __AWS_H__

#ifdef    __cplusplus
extern "C" {
#endif

static long WlanConnect(const char *ssid);
static int set_time();
static long InitializeAppVariables();
static int tls_connect();
static int connectToAccessPoint(const char *ssid);
int ConnectToEndpoint(const char *ssid);
int http_post(int, char*, char*, int);

void craftRequest(char *request, float az, float alt, float roll, int minMag, int utc_offset);      // utc_offset in seconds

#ifdef  __cplusplus
}
#endif /* __cplusplus */

#endif

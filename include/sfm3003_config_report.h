// See https://gcc.gnu.org/onlinedocs/cpp/Stringizing.html and https://gcc.gnu.org/onlinedocs/cpp/Concatenation.html
// Note that the actual #defines in sdkconfig.h are prefixed by "CONFIG_" relative to the declaration in KConfig.
#ifndef CONFIG_LI
#define STR(X) #X
#define XSTR(X) STR(X)
#define CONFIG_LI(Y) "<li>" #Y " = " XSTR(CONFIG_##Y) "</li>\n"
#endif

// ignoring the config which defines the defaults for settings, as this info is already revealed.
const char* sfm3003_config =  "<ul>" CONFIG_LI(SFM3003_LOG_LEVEL) CONFIG_LI(SFM_LP_BUFF_LEN) "</ul>\n";
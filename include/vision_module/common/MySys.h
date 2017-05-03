
#ifndef _MYSYS
#define _MYSYS

#include <limits.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <algorithm>

#define ALL 1

int myExec(const char *command);
char *myCWD(char *path, const unsigned int &size);
int myCD(const char *path);
int myMKDIR(const char *dirName, mode_t mode);
int myRMDIR(const char *dirName);
int myMV(const char *src, const char *dst);
int myCP(const char *src, const char *dst);
std::vector<std::string> myDIRENT(const char *dirName, 
                                            const unsigned char type = 0);
#endif

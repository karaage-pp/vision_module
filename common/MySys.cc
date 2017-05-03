
#include <vision_module/common/MySys.h>
#include <string.h>

int myExec(const char *command){
    int ret;
    ret = system(command);
    return WEXITSTATUS(ret);
}

char *myCWD(char *path, const unsigned int &size){
    return getcwd(path, size);
}

int myCD(const char *path){
    return chdir(path);
}

int myMKDIR(const char *dirName, mode_t mode){
    return mkdir(dirName, mode);
}

int myRMDIR(const char *dirName){
    return rmdir(dirName);
}

int myMV(const char *src, const char *dst){
    int ret;
    char command[512] = "";

    sprintf(command, "mv %s %s", src, dst);
    ret = system(command);
    return WEXITSTATUS(ret);
}

int myCP(const char *src, const char *dst){
    int ret;
    char command[512] = "";

    sprintf(command, "cp %s %s", src, dst);
    ret = system(command);
    return WEXITSTATUS(ret);
}

std::vector<std::string> myDIRENT(const char *dirName, 
                                    const unsigned char type){
    std::vector<std::string> res;
    res.clear();

    DIR *dp = opendir(dirName);
    if(dp){
        struct dirent *dent;
        do{
            dent = readdir(dp);
            if(dent){
                if(type == 0)
                    if(dent->d_name[0] == '.')continue;
                res.push_back(dent->d_name);
            }
        }while(dent);
        closedir(dp);
    }
    std::sort(res.begin(), res.end());
    return res;
}


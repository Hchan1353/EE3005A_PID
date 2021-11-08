#include<remap.h>
int remapPIDOut(int pidOut){
    return ((double)pidOut/255.0)*180.0;
}

int remapPIDIn(int pidIn){
    return pidIn+90;
}
#ifndef PCS_H
#define PCS_H

#include "common.h"

class PCS
{
public:
    PCS();
    PCS(vector<string> files_pcd_);
    PCS(vector<string> files_pcd_, vector<string> files_label_);
//    bool loadFromFile(string file);
    CloudTPtr pcRaw(int n);
    CloudTPtr pcLabel(int n);

private:
    vector<string> split(string source, char delim, int rep=0);
    void push(double t, PointTT point);

public:
    vector<CloudTPtr> pcs;
    CloudTPtr pc_raw;
    CloudTPtr pc_label;
    int nTime;
    vector<string> files_pcd, files_label;

private:
    int i;
    int cnt;

    int maxID;
    int *r, *g, *b;
};

#endif // PCS_H

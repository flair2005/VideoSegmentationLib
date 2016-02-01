#include "pcs.h"

PCS::PCS()
{
    cnt = 0;
}

PCS::PCS(vector<string> files_pcd_)
{
    cnt = 0;
    files_pcd = files_pcd_;
    nTime = files_pcd.size();
}

PCS::PCS(vector<string> files_pcd_, vector<string> files_label_)
{
    cnt = 0;
    files_pcd = files_pcd_;
    files_label = files_label_;
    nTime = files_pcd.size();

    maxID = 100;
    srand(time(NULL));
    r = new int[maxID];
    g = new int[maxID];
    b = new int[maxID];

    for(int i=0;i<maxID;i++){
        r[i] = rand() % 256;
        g[i] = rand() % 256;
        b[i] = rand() % 256;
    }
}

CloudTPtr PCS::pcRaw(int n)
{
    string file = files_pcd.at(n);
    pc_raw.reset(new CloudT);
    pcl::io::loadPCDFile<PointTT>(file.c_str(), *pc_raw);
    return  pc_raw;
}

CloudTPtr PCS::pcLabel(int n)
{
    string file = files_label.at(n);
    pc_label.reset(new CloudT);

    ifstream stm;
    stm.open (file.c_str());
    vector<int> labels;
    string word;
    while (stm >> word) labels.push_back(atoi(word.c_str()));
    for(int i=0;i<pc_raw->points.size();i++){
        PointTT pt = pc_raw->points.at(i);
        int label;
        if(i < labels.size()){
            label = labels.at(i);
            pt.r = r[label];
            pt.g = g[label];
            pt.b = b[label];
        }
        else{
            pt.r = 0;
            pt.g = 0;
            pt.b = 0;
        }

        pc_label->points.push_back(pt);
    }
    pc_label->header = pc_raw->header;
    pc_label->width = pc_raw->width;
    pc_label->height = pc_raw->height;
    pc_label->is_dense = pc_raw->is_dense;
    pc_label->sensor_orientation_ = pc_raw->sensor_orientation_;
    pc_label->sensor_origin_ = pc_raw->sensor_origin_;
    return  pc_label;
}

//bool PCS::loadFromFile(string file)
//{
//    cout<<"loading... "<<endl;
//    string line;
//    ifstream myfile (file.c_str());
//    if (myfile.is_open()){
//        pc.reset(new Cloud);
//        while ( getline (myfile,line) ){
//            vector<string> flds = split(line, ' ');
//            PointT point;
//            double t = QString(flds.at(0).c_str()).toDouble();
//            point.x = QString(flds.at(1).c_str()).toDouble();
//            point.y = QString(flds.at(2).c_str()).toDouble();
//            point.z = QString(flds.at(3).c_str()).toDouble();
//            point.r = QString(flds.at(4).c_str()).toDouble();
//            point.g = QString(flds.at(5).c_str()).toDouble();
//            point.b = QString(flds.at(6).c_str()).toDouble();

//            /*double t = stod(flds.at(0));
//            point.x = stod(flds.at(1));
//            point.y = stod(flds.at(2));
//            point.z = stod(flds.at(3));
//            point.r = stod(flds.at(4));
//            point.g = stod(flds.at(5));
//            point.b = stod(flds.at(6));*/

//            if(t == cnt){
//                pc->points.push_back(point);
//            }
//            else if(t == cnt+1){
//                // at new time
//                pcs.push_back(pc);
//                pc.reset(new Cloud);
//                pc->points.push_back(point);
//                cnt = cnt+1;
//            }
//        }
//        pcs.push_back(pc);
//        nTime = pcs.size();
//        myfile.close();
//    }
//    else return 0;

//    return 1;
//}

vector<string> PCS::split(string source, char delim, int rep) {
    vector<string> flds;
    string work = source;
    string buf = "";
    int i = 0;
    while (i < work.length()) {
        if (work[i] != delim)
            buf += work[i];
        else if (rep == 1) {
            flds.push_back(buf);
            buf = "";
        } else if (buf.length() > 0) {
            flds.push_back(buf);
            buf = "";
        }
        i++;
    }
    if (!buf.empty())
        flds.push_back(buf);
    return flds;
}

void PCS::push(double t, PointTT point)
{

}

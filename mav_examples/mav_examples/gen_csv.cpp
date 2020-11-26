#include <iostream>
#include <fstream>
using namespace std;
void create(){
    fstream fout;
    int n;
    fout.open("traverse_points.csv", ios::out | ios::app );
    cout<<"Enter the no of points to be traversed \n";
    cin>>n;
    cout<<"Enter the points in format x y z :\n";
    int i,x,y,z;
    for(i=0;i<n;i++){
        cin>>x>>y>>z;
        fout<<i<<", "
            <<x<<", "
            <<y<<", "
            <<z<<"\n";
    }
}
int main(){create();return 0;}

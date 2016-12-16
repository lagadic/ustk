#ifndef US_SCAN_CONVERTER_3D_H
#define US_SCAN_CONVERTER_3D_H

#include <cmath>
#include <vector>

#include <visp3/ustk_core/usImagePreScan3D.h>
#include <visp3/ustk_core/usImagePostScan3D.h>


class VISP_EXPORT usScanConverter3D
{
protected:
    double DEFAULT_DEG_PER_LINE = 0.608767657441;
    double DEFAULT_PROBE_RADIUS = 0.04;
    double DEFAULT_MOTOR_RADIUS = 0.02725;
    double DEFAULT_CENTER_OFFSET = DEFAULT_PROBE_RADIUS - DEFAULT_MOTOR_RADIUS;
    double SPEED_OF_SOUND = 1540;
    double DEFAULT_BSAMPLE_DISTANCE = 0.000308;
    double DEFAULT_DEG_PER_FRAME = 1.463;
    double DEFAULT_RAD_PER_FRAME = M_PI / 180.0 * DEFAULT_DEG_PER_FRAME;
    double DEFAULT_RAD_PER_LINE = M_PI / 180.0 * DEFAULT_DEG_PER_LINE;

    class VoxelWeightAndIndex
    {
        friend class usScanConverter3D;
        unsigned int _outputIndex;
        unsigned int _inputIndex[8];
        double _W[8];
    };

    std::vector<VoxelWeightAndIndex> _lookupTable1;
    std::vector<VoxelWeightAndIndex> _lookupTable2;

    usImagePreScan3D<unsigned char> _VpreScan;
    usImagePostScan3D<unsigned char> _VpostScan;

    double _resolution;
    bool _SweepInZdirection;

public:

    //! Constructors, destructors

    usScanConverter3D();
    usScanConverter3D(int x, int y, int z, int down = 1);
    virtual ~usScanConverter3D();

    void init(int X, int Y, int Z, int down = 1);

    double getResolution() const;

    void setVolume(const usImagePreScan3D<unsigned char> &V);

    void getVolume(usImagePostScan3D<unsigned char> &V);
    usImagePostScan3D<unsigned char> getVolume();

    double getResolution() {return _resolution;}

    void SweepInZdirection(bool flag) {_SweepInZdirection = flag;}

    void convert();

    void convertPreScanCoordToPostScanCoord(double i, double j, double k, double *x=NULL, double *y=NULL, double *z=NULL, bool sweepInZdirection=true);
    void convertPostScanCoordToPreScanCoord(double x, double y, double z, double *i=NULL, double *j=NULL, double *k=NULL, bool sweepInZdirection=true);
};

#endif // US_SCAN_CONVERTER_3D_H

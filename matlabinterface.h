#ifndef MATLABINTERFACE
#define MATLABINTERFACE

#include <QFile>
#include <QString>
#include <QDataStream>

struct snakeSectionData
{
    float x;
    float y;
    float phi;
    float dx;
    float dy;
    float d_phi;
    float f_res_x;
    float f_res_y;
    float torque;
};

struct snakeMCPos
{
    float t;
    float x;
    float y;
};

struct interfaceData
{
    volatile quint32 turn;
    quint32 readHeartBeat;
    volatile quint32 writeHeartBeat;
    volatile quint32 msgWritten;
    quint32 msgRead;
    quint32 iteration;
    quint32 numSections;
    float headPosX;
    float headPosY;
    float headAngle;
    snakeSectionData section[100];
};

struct fileRecord
{
    float               t;
    float               headPosX;
    float               headPosY;
    float               headAngle;
};

class matlabFileInterface
{
private:
    quint32 N;
    quint32 numberOfSamples;
    std::vector<fileRecord> position;
    std::vector<snakeSectionData> sections;
    std::vector<snakeMCPos> mcposition;
    QFile file;
    quint32 it;
    float time;

    struct interpolationParameters
    {
        int i1;
        int i2;
        float scale;
    };

    interpolationParameters getInterpolationParameters()
    {
        int i1,i2;
        const float rho = 0.001f; // Simply snap to a sample if time is within this distance
        // First figure out which two samples to interpolate from
        if(time < position[it].t + rho && time > position[it].t - rho)
        {
            i1 = it;
            i2 = it;
        }
        else if(position[it].t < time)
        {
            i1 = it;
            if(it == numberOfSamples-1)
            {
                i2 = it;
            }
            else
            {
                i2 = i1+1;
            }
        }
        else if(position[it].t > time)
        {
            i2 = it;
            if(it == 0)
            {
                i1 = it;
            }
            else
            {
                i1 = i2-1;
            }
        }
        float t1 = position[i1].t;
        float t2 = position[i2].t;
        float dt = t2-t1;
        float scale;
        if(dt < rho)
        {
            scale = 0.0f;
        }
        else
        {
            scale = (time-t1)/dt;
        }
        interpolationParameters r;
        r.i1 = i1;
        r.i2 = i2;
        r.scale = scale;
        return r;
    }

public:
    matlabFileInterface(QString fileName) :
        position(),
        sections(),
        file(fileName),
        it(0)
    {
        if(file.exists())
        {
            file.open(QIODevice::ReadOnly);
            QDataStream in(&file);
            in.setByteOrder(QDataStream::LittleEndian);
            in >> N;
            in >> numberOfSamples;
            in.setFloatingPointPrecision(QDataStream::SinglePrecision);
            for(quint32 i = 0; i < numberOfSamples; ++i)
            {
                fileRecord r;
                in >> r.t;
                in >> r.headPosX;
                in >> r.headPosY;
                in >> r.headAngle;
                position.push_back(r);
                for(quint32 j = 0; j < N; ++j)
                {
                    snakeSectionData s;
                    in >> s.x;
                    in >> s.y;
                    in >> s.phi;
                    in >> s.dx;
                    in >> s.dy;
                    in >> s.d_phi;
                    in >> s.f_res_x;
                    in >> s.f_res_y;
                    in >> s.torque;
                    sections.push_back(s);
                }
            }
        }
    }
    int getNumberOfSections()
    {
        return N;
    }
    int getNumberOfSamples()
    {
        return numberOfSamples;
    }

    void iterateToClosestTimePoint(float t)
    {
        time = t;
        if(t > position[it].t && t < numberOfSamples-1 && !(t < position[it+1].t))
        {
            while(t > position[it].t && it < numberOfSamples)
            {
                ++it;
            }
        }
        else if(t < position[it].t && t > 0 && !(t > position[it-1].t))
        {
            while(t < position[it].t && it > 0)
            {
                --it;
            }
        }
    }

    bool next()
    {
        if(it < numberOfSamples-1)
        {
            ++it;
            time = position[it].t;
            return true;
        }
        return false;
    }

    void reset()
    {
        it = 0;
    }
    snakeSectionData getSection(int s)
    {
        interpolationParameters p = getInterpolationParameters();
        int i1 = p.i1;
        int i2 = p.i2;
        float scale = p.scale;
        snakeSectionData d;
        d.x =           sections[i1*N+s].x         + scale*(sections[i2*N+s].x          - sections[i1*N+s].x);
        d.y =           sections[i1*N+s].y         + scale*(sections[i2*N+s].y          - sections[i1*N+s].y);
        d.phi =         sections[i1*N+s].phi       + scale*(sections[i2*N+s].phi        - sections[i1*N+s].phi);
        d.dx =          sections[i1*N+s].dx        + scale*(sections[i2*N+s].dx         - sections[i1*N+s].dx);
        d.dy =          sections[i1*N+s].dy        + scale*(sections[i2*N+s].dy         - sections[i1*N+s].dy);
        d.d_phi =       sections[i1*N+s].d_phi     + scale*(sections[i2*N+s].d_phi      - sections[i1*N+s].d_phi);
        d.f_res_x =     sections[i1*N+s].f_res_x   + scale*(sections[i2*N+s].f_res_x    - sections[i1*N+s].f_res_x);
        d.f_res_y =     sections[i1*N+s].f_res_y   + scale*(sections[i2*N+s].f_res_y    - sections[i1*N+s].f_res_y);
        d.torque =      sections[i1*N+s].torque    + scale*(sections[i2*N+s].torque     - sections[i1*N+s].torque);


        return d;
    }
    float get_time()
    {
        return time;
    }
    float get_lastTime()
    {
        return position[numberOfSamples-1].t;
    }

    float get_headX()
    {
        interpolationParameters p = getInterpolationParameters();
        return position[p.i1].headPosX + p.scale*(position[p.i2].headPosX-position[p.i1].headPosX);
    }
    float get_headY()
    {
        interpolationParameters p = getInterpolationParameters();
        return position[p.i1].headPosY + p.scale*(position[p.i2].headPosY-position[p.i1].headPosY);
    }
    float get_headAngle()
    {
        interpolationParameters p = getInterpolationParameters();
        return position[p.i1].headAngle + p.scale*(position[p.i2].headAngle-position[p.i1].headAngle);
    }
};

class matlabSharedMemoryInterface
{
public:
    matlabSharedMemoryInterface(QString fileName) :
        mappedFile(fileName),
        copy(),
        notConnected(true),
        iterationsWithoutWriteHeartbeat(0)
    {
        if(!mappedFile.exists())
        {
            mappedFile.open(QIODevice::ReadWrite);
            mappedFile.setPermissions(QFileDevice::WriteOther | QFileDevice::ReadOther);
            mappedFile.resize(sizeof(interfaceData));
            mappedFile.close();
        }
        mappedFile.open(QIODevice::ReadWrite);
        mappedFile.setPermissions(QFileDevice::WriteOther | QFileDevice::ReadOther);
        file = mappedFile.map(0,mappedFile.size());
        mappedFile.close();
        if(!file)
        {
            // File not successfully mapped
        }
        else
        {
            reinterpret_cast<interfaceData*>(file)->turn = 0;
            reinterpret_cast<interfaceData*>(file)->msgRead = true;
            reinterpret_cast<interfaceData*>(file)->readHeartBeat++;
            lastWriteHeartBeat = reinterpret_cast<interfaceData*>(file)->writeHeartBeat;
            lastIteration = reinterpret_cast<interfaceData*>(file)->iteration;
        }
    }
    ~matlabSharedMemoryInterface()
    {
        mappedFile.unmap(file);
    }

    bool readData()
    {
        reinterpret_cast<interfaceData*>(file)->readHeartBeat++;    // This process is listening to matlab
        quint32 whb = reinterpret_cast<interfaceData*>(file)->writeHeartBeat;
        // Find out if matlab side is connected or not.
        if(lastWriteHeartBeat == whb)
        {
            iterationsWithoutWriteHeartbeat++;
            if(iterationsWithoutWriteHeartbeat > 100)
            {
                notConnected = true;
            }
            return false;
        }
        else
        {
            notConnected = false;
            iterationsWithoutWriteHeartbeat = 0;
            lastWriteHeartBeat = whb;
        }

        // Entering read when condition is met
        while(!reinterpret_cast<interfaceData*>(file)->msgWritten ||
              reinterpret_cast<interfaceData*>(file)->turn == 0);
        reinterpret_cast<interfaceData*>(file)->msgRead = false;    // Gains exclusive access to data
        if(reinterpret_cast<interfaceData*>(file)->iteration == lastIteration) // Cancel update if nothing happened
        {
            reinterpret_cast<interfaceData*>(file)->msgRead = true;
            return false;
        }
        lastIteration = reinterpret_cast<interfaceData*>(file)->iteration;
        copy = *reinterpret_cast<interfaceData*>(file);
        reinterpret_cast<interfaceData*>(file)->msgRead = true;
        reinterpret_cast<interfaceData*>(file)->turn = 0;

        return true;
    }

    bool isConnectionProbablyMissing()
    {
        return notConnected;
    }

    int getNumberOfSections()
    {
        return copy.numSections;
    }
    snakeSectionData getSection(int s)
    {

        return copy.section[s];
    }

    int getIteration()
    {
        return copy.iteration;
    }

    int getReadHeartbeat()
    {
        return reinterpret_cast<interfaceData*>(file)->readHeartBeat;
    }
    int getWriteHeartbeat()
    {
        return reinterpret_cast<interfaceData*>(file)->writeHeartBeat;
    }
    int getTurn()
    {
        return reinterpret_cast<interfaceData*>(file)->turn;
    }
    float get_headX()
    {
        return copy.headPosX;
    }
    float get_headY()
    {
        return copy.headPosY;
    }
    float get_headAngle()
    {
        return copy.headAngle;
    }

    bool messageWritten()
    {
        return reinterpret_cast<interfaceData*>(file)->msgWritten;
    }
    bool messageRead()
    {
        return reinterpret_cast<interfaceData*>(file)->msgRead;
    }

private:

    QFile mappedFile;
    uchar * file;
    interfaceData copy;
    int lastIteration;
    int lastWriteHeartBeat;
    bool notConnected;
    int iterationsWithoutWriteHeartbeat;


};

#endif // MATLABINTERFACE


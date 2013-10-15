/*
Copyright (c) 2003-2010 Sony Pictures Imageworks Inc., et al.
All Rights Reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
* Neither the name of Sony Pictures Imageworks nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdint.h>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iterator>
#include <sstream>

#include <OpenColorIO/OpenColorIO.h>

#include "FileTransform.h"
#include "Lut1DOp.h"
#include "Lut3DOp.h"
#include "MathUtils.h"
#include "ParseUtils.h"
#include "pystring/pystring.h"

OCIO_NAMESPACE_ENTER
{
namespace
{
    struct FileHeader
    {
        uint32_t magic; // 4Bytes, must be 0x42340299
        uint32_t ver; // 4Bytes, 0x01000002
        char model[16]; // 16Bytes, monitor model. No match required for DIT LUT.
        char version[16]; // 16Bytes, data version, eg. “1.0.11”
        uint32_t data_checksum; // 4Bytes, data sum
        uint32_t length; // 4Bytes, data length = 1048576
        char description[16]; //16Bytes, 3dlut description info, e.g. “LightSpace(c)”
        char reserved[63]; // reserved
        uint8_t header_checksum; // file header sum
    };

    uint8_t ComputeHeaderChecksum(const FileHeader & header)
    {
        const uint8_t * buf = reinterpret_cast<const uint8_t *>(&header);
        uint8_t header_sum = 0;
        for(size_t i=0; i<(sizeof(FileHeader)-1); ++i)
        {
            header_sum = (uint8_t) (int(header_sum) + int(buf[i]));
        }
        return header_sum;
    }

    uint32_t ComputeDataChecksum(uint8_t * buf, size_t size)
    {
        uint32_t data_sum = 0;
        for(size_t i=0; i<size; ++i)
        {
            data_sum += buf[i];
        }
        return data_sum;
    }

    // The spec specifically calls out 1008 as the 10-bit int maxval.
    inline float flanders_decode_r(uint32_t rgb)
    {
        return float(rgb & 0x3ff) / 1008.f;
    }

    inline float flanders_decode_g(uint32_t rgb)
    {
        return float((rgb & 0xffc00) >> 10) / 1008.f;
    }

    inline float flanders_decode_b(uint32_t rgb)
    {
        return float((rgb & 0x3ff00000) >> 20) / 1008.f;
    }

    inline uint32_t flanders_encode(float r, float g, float b)
    {
        uint32_t rint = (uint32_t) roundf( std::min(1.0f, std::max(0.0f, r)) * 1008.f);
        uint32_t gint = (uint32_t) roundf( std::min(1.0f, std::max(0.0f, g)) * 1008.f);
        uint32_t bint = (uint32_t) roundf( std::min(1.0f, std::max(0.0f, b)) * 1008.f);
        return (rint | (gint << 10) | (bint << 20));
    }

    class CachedFileFSI : public CachedFile
    {
    public:
        CachedFileFSI() : lut3D()
        {
        };
        ~CachedFileFSI() {};
        Lut3DRcPtr lut3D;
    };
    typedef OCIO_SHARED_PTR<CachedFileFSI> CachedFileFSIRcPtr;



    class LocalFileFormat : public FileFormat
    {
    public:
        ~LocalFileFormat() {};

        virtual void GetFormatInfo(FormatInfoVec & formatInfoVec) const;
        
        virtual CachedFileRcPtr Read(std::istream & istream) const;
        
        virtual void Write(const Baker & baker,
                           const std::string & formatName,
                           std::ostream & ostream) const;
        
        virtual void BuildFileOps(OpRcPtrVec & ops,
                                  const Config& config,
                                  const ConstContextRcPtr & context,
                                  CachedFileRcPtr untypedCachedFile,
                                  const FileTransform& fileTransform,
                                  TransformDirection dir) const;
    };

    void LocalFileFormat::GetFormatInfo(FormatInfoVec & formatInfoVec) const
    {
        FormatInfo info;
        info.name = "flanders";
        info.extension = "dat";
        info.capabilities = (FORMAT_CAPABILITY_READ | FORMAT_CAPABILITY_WRITE);
        formatInfoVec.push_back(info);
    }

    CachedFileRcPtr LocalFileFormat::Read(std::istream & istream) const
    {
        if (!istream)
        {
            throw Exception ("file stream empty when trying to Flanders lut");
        }

        FileHeader header;
        istream.read((char*)(&header), sizeof(header));
        if(!istream.good())
        {
            std::ostringstream os;
            os << "Error reading header for Flanders lut.";
            throw Exception(os.str().c_str());
        }

        if(header.magic != 0x42340299)
        {
            std::ostringstream os;
            os << "Flanders Lut is not valid: ";
            os << "expected magic number '1110704793', found " << header.magic;
            throw Exception(os.str().c_str());
        }

        uint8_t headerChecksum = ComputeHeaderChecksum(header);
        if(headerChecksum != header.header_checksum)
        {
            std::ostringstream os;
            os << "Flanders Lut is not valid: ";
            os << "Header checksum '" << header.header_checksum << "' ";
            os << "does not match computed value '" << headerChecksum << "'.";
            throw Exception(os.str().c_str());
        }

        std::vector<uint8_t> data(header.length, 0);
        istream.read((char*)(&data[0]), header.length);
        if(!istream.good())
        {
            std::ostringstream os;
            os << "Error reading lut data for Flanders lut.";
            throw Exception(os.str().c_str());
        }

        uint32_t dataChecksum = ComputeDataChecksum(&data[0], header.length);
        if(dataChecksum != header.data_checksum)
        {
            std::ostringstream os;
            os << "Flanders Lut is not valid: ";
            os << "data checksum '" << header.data_checksum << "' ";
            os << "does not match computed value '" << dataChecksum << "'.";
            throw Exception(os.str().c_str());
        }

        Lut3DRcPtr lut3d_ptr = Lut3D::Create();
        int numEntries = header.length/4;
        int edgeLen = Get3DLutEdgeLenFromNumPixels(numEntries);
        lut3d_ptr->lut.resize(numEntries * 3);
        lut3d_ptr->size[0] = edgeLen;
        lut3d_ptr->size[1] = edgeLen;
        lut3d_ptr->size[2] = edgeLen;

        uint32_t * dataptr = (uint32_t *)(&data[0]);
        for (int i=0; i<numEntries; ++i)
        {
            lut3d_ptr->lut[3*i+0] = flanders_decode_r(dataptr[i]);
            lut3d_ptr->lut[3*i+1] = flanders_decode_g(dataptr[i]);
            lut3d_ptr->lut[3*i+2] = flanders_decode_b(dataptr[i]);
        }

        CachedFileFSIRcPtr cachedFile = CachedFileFSIRcPtr(new CachedFileFSI());
        cachedFile->lut3D = lut3d_ptr;
        return cachedFile;
    }

    void LocalFileFormat::Write(const Baker & baker,
                       const std::string & /*formatName*/,
                       std::ostream & ostream) const
    {
        int DEFAULT_CUBE_SIZE = 64;

        ConstConfigRcPtr config = baker.getConfig();

        int cubeSize = baker.getCubeSize();
        if(cubeSize==-1) cubeSize = DEFAULT_CUBE_SIZE;
        cubeSize = std::max(2, cubeSize); // smallest cube is 2x2x2

        std::vector<float> cubeData;
        cubeData.resize(cubeSize*cubeSize*cubeSize*3);
        GenerateIdentityLut3D(&cubeData[0], cubeSize, 3, LUT3DORDER_FAST_RED);
        PackedImageDesc cubeImg(&cubeData[0], cubeSize*cubeSize*cubeSize, 1, 3);

        // Apply our conversion from the input space to the output space.
        ConstProcessorRcPtr inputToTarget;
        std::string looks = baker.getLooks();
        if (!looks.empty())
        {
            LookTransformRcPtr transform = LookTransform::Create();
            transform->setLooks(looks.c_str());
            transform->setSrc(baker.getInputSpace());
            transform->setDst(baker.getTargetSpace());
            inputToTarget = config->getProcessor(transform,
                TRANSFORM_DIR_FORWARD);
        }
        else
        {
            inputToTarget = config->getProcessor(baker.getInputSpace(),
                  baker.getTargetSpace());
        }
        inputToTarget->apply(cubeImg);

        std::vector<uint32_t> encodeddata(cubeSize*cubeSize*cubeSize);
        for(int i=0; i<cubeSize*cubeSize*cubeSize; ++i)
        {
            encodeddata[i] = flanders_encode(
                cubeData[3*i+0], cubeData[3*i+1], cubeData[3*i+2]);
        }
        uint8_t * buf = (uint8_t *)(&encodeddata[0]);
        size_t bufsize = encodeddata.size()*sizeof(uint32_t);

        // Write out the output data.
        // Null out full header struct to assure that all values are cleared
        // (Reserved, etc)
        FileHeader header;
        memset(&header, 0, sizeof(header));
        header.magic = 0x42340299;
        header.ver = 0x1000001; // Or should this be 0x01000002?
        //strncpy(header.model, "", 16);
        if(baker.getMetadata())
        {
            strncpy(header.model, baker.getMetadata(), 16);
        }
        strncpy(header.version, "1.0.11", 16);
        header.data_checksum = ComputeDataChecksum(buf, bufsize);
        header.length = (uint32_t) bufsize;
        strncpy(header.description, "OpenColorIO", 16);
        
        header.header_checksum = ComputeHeaderChecksum(header);
        ostream.write((const char *)(&header), sizeof(FileHeader));
        ostream.write((const char *)buf, bufsize);
    }

    void LocalFileFormat::BuildFileOps(OpRcPtrVec & ops,
                              const Config& /*config*/,
                              const ConstContextRcPtr & /*context*/,
                              CachedFileRcPtr untypedCachedFile,
                              const FileTransform& fileTransform,
                              TransformDirection dir) const
    {
        CachedFileFSIRcPtr cachedFile = DynamicPtrCast<CachedFileFSI>(untypedCachedFile);

        // This should never happen.
        if(!cachedFile)
        {
            std::ostringstream os;
            os << "Cannot build FSI Op. Invalid cache type.";
            throw Exception(os.str().c_str());
        }

        TransformDirection newDir = CombineTransformDirections(dir,
            fileTransform.getDirection());
        CreateLut3DOp(ops, cachedFile->lut3D,
                      fileTransform.getInterpolation(), newDir);
    }
}

FileFormat * CreateFileFormatFlanders()
{
    return new LocalFileFormat();
}

}
OCIO_NAMESPACE_EXIT



///////////////////////////////////////////////////////////////////////////////

#ifdef OCIO_UNIT_TEST

namespace OCIO = OCIO_NAMESPACE;
#include "UnitTest.h"

#include <iostream>
#include <fstream>

#if 0
OIIO_ADD_TEST(FileFormatFlanders, test3d)
{
    std::ifstream filestream("/net/homedirs/jeremys/git/ocio.js/dit.dat");
    OCIO::LocalFileFormat format;
    OCIO::CachedFileRcPtr lut = format.Read(filestream);
}
#endif

#endif // OCIO_UNIT_TEST


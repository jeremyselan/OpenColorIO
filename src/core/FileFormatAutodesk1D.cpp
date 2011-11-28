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

#include <OpenColorIO/OpenColorIO.h>

#include "FileTransform.h"
#include "Lut1DOp.h"
#include "Lut3DOp.h"
#include "MathUtils.h"
#include "ParseUtils.h"
#include "pystring/pystring.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <iostream>

/*
// Autodesk's 1D format

Example:

# Autodesk 1D Export from Truelight v4.0 title{TestTitle} 
LUT: 3 1024
  0
  1
  2
  3
...
  1022
  1023
  0
  1
...
  1022
  1023
  0
  1
...
  1022
  1023
*/


OCIO_NAMESPACE_ENTER
{
    ////////////////////////////////////////////////////////////////
    
    namespace
    {
        class LocalCachedFile : public CachedFile
        {
        public:
            LocalCachedFile ()
            {
                lut1D = OCIO_SHARED_PTR<Lut1D>(new Lut1D());
            };
            ~LocalCachedFile() {};
            
            OCIO_SHARED_PTR<Lut1D> lut1D;
        };
        
        typedef OCIO_SHARED_PTR<LocalCachedFile> LocalCachedFileRcPtr;
        
        
        
        class LocalFileFormat : public FileFormat
        {
        public:
            
            ~LocalFileFormat() {};
            
            virtual void GetFormatInfo(FormatInfoVec & formatInfoVec) const;
            
            virtual CachedFileRcPtr Read(std::istream & istream) const;
            
            virtual void BuildFileOps(OpRcPtrVec & ops,
                                      const Config& config,
                                      const ConstContextRcPtr & context,
                                      CachedFileRcPtr untypedCachedFile,
                                      const FileTransform& fileTransform,
                                      TransformDirection dir) const;
        };
        
        int GetMaxValueFromIntegerBitDepth(int bitDepth)
        {
            return static_cast<int>( pow(2.0, bitDepth) ) - 1;
        }
        
        int GetClampedIntFromNormFloat(float val, float scale)
        {
            val = std::min(std::max(0.0f, val), 1.0f) * scale;
            return static_cast<int>(roundf(val));
        }
        
        void LocalFileFormat::GetFormatInfo(FormatInfoVec & formatInfoVec) const
        {
            FormatInfo info;
            info.name = "autodesk-1d";
            info.extension = "lut";
            info.capabilities = FORMAT_CAPABILITY_READ;
            formatInfoVec.push_back(info);
        }
        
        // Try and load the format
        // Raise an exception if it can't be loaded.
        
        CachedFileRcPtr LocalFileFormat::Read(std::istream & istream) const
        {
            std::vector<int> rawdata;
            int numchannels = 0;
            int numentries = 0;
            
            // Parse the file 3d lut data to an int array
            {
                const int MAX_LINE_SIZE = 4096;
                char lineBuffer[MAX_LINE_SIZE];
                
                std::vector<std::string> lineParts;
                
                while(istream.good())
                {
                    istream.getline(lineBuffer, MAX_LINE_SIZE);
                    
                    // Strip and split the line
                    pystring::split(pystring::strip(lineBuffer), lineParts);
                    
                    if(lineParts.empty()) continue;
                    if((lineParts.size() > 0) && pystring::startswith(lineParts[0],"#")) continue;
                    
                    // Look for the LUT: tag
                    if((lineParts.size() == 3) && (pystring::lower(lineParts[0]) == "lut:"))
                    {
                        if(!StringToInt(&numchannels, lineParts[1].c_str()) ||
                           !StringToInt(&numentries, lineParts[2].c_str()))
                        {
                            std::ostringstream os;
                            os << "Error parsing Autodesk 1d LUT.";
                            os << "Expected 'LUT: %d %d', found ";
                            os << lineBuffer;
                            throw Exception(os.str().c_str());
                        }
                        
                        if(numchannels<=0 || numchannels>4)
                        {
                            std::ostringstream os;
                            os << "Error parsing Autodesk 1d LUT.";
                            os << "Numchannels must be from 1-4.";
                            throw Exception(os.str().c_str());
                        }
                        if(numentries<=0)
                        {
                            std::ostringstream os;
                            os << "Error parsing Autodesk 1d LUT.";
                            os << "numentries must be positive.";
                            throw Exception(os.str().c_str());
                        }
                        
                        continue;
                    }
                    
                    // If we havent found an int, continue
                    int tmpint = 0;
                    if(!StringToInt(&tmpint, lineParts[0].c_str())) continue;
                    
                    rawdata.push_back(tmpint);
                }
            }
            
            std::cerr << "numchannels " << numchannels << std::endl;
            std::cerr << "numentries " << numentries << std::endl;
                
              
            if(rawdata.empty())
            {
                std::ostringstream os;
                os << "Error parsing Autodesk 1d LUT.";
                os << "Does not appear to contain valid lut data.";
                throw Exception(os.str().c_str());
            }
            std::cerr << "rawdata.size() " << rawdata.size() << std::endl;
            
            LocalCachedFileRcPtr cachedFile = LocalCachedFileRcPtr(new LocalCachedFile());
            
            if(static_cast<int>(rawdata.size()) != numchannels*numentries)
            {
                std::ostringstream os;
                os << "Error parsing Autodesk 1d LUT.";
                os << "Found " << rawdata.size() << " data entries,";
                os << " but expected " << (numchannels*numentries);
                throw Exception(os.str().c_str());
            }
            
            // int numchannels = 0;
            // int numentries = 0;
            
            // Process the 1d lut data
            
            const int FORMATLUT_CODEVALUE_LOWEST_PLAUSIBLE_MAXINT = 128;
            {
                // Find the maximum shaper lut value to infer bit-depth
                int shapermax = 0;
                for(unsigned int i=0; i<rawdata.size(); ++i)
                {
                    shapermax = std::max(shapermax, rawdata[i]);
                }
                
                if(shapermax<FORMATLUT_CODEVALUE_LOWEST_PLAUSIBLE_MAXINT)
                {
                    std::ostringstream os;
                    os << "Error parsing Autodesk 1d LUT.";
                    os << "The maximum lut value, " << shapermax;
                    os << ", is unreasonably low. This lut is probably not a .3dl ";
                    os << "file, but instead a related format that shares a similar ";
                    os << "structure.";
                    
                    throw Exception(os.str().c_str());
                }
                
                int bitdepthmax = numentries-1;
                float scale = 1.0f / static_cast<float>(bitdepthmax);
                
                std::cerr << "Scale " << scale << std::endl;
                
                for(int channel=0; channel<3; ++channel)
                {
                    cachedFile->lut1D->luts[channel].reserve(numentries);
                    
                    int chanindex = std::min((int)channel, numchannels);
                    
                    for(int i=0; i<numentries; ++i)
                    {
                        int ival = rawdata[chanindex*numentries+i];
                        float fval = static_cast<float>(ival)*scale;
                        cachedFile->lut1D->luts[channel].push_back(fval);
                    }
                }
                
                const int FORMATLUT_SHAPER_CODEVALUE_TOLERANCE = 2;
                float error = FORMATLUT_SHAPER_CODEVALUE_TOLERANCE*scale;
                
                cachedFile->lut1D->finalize(error, ERROR_ABSOLUTE);
            }
            
            return cachedFile;
        }
        
        void
        LocalFileFormat::BuildFileOps(OpRcPtrVec & ops,
                                      const Config& /*config*/,
                                      const ConstContextRcPtr & /*context*/,
                                      CachedFileRcPtr untypedCachedFile,
                                      const FileTransform& fileTransform,
                                      TransformDirection dir) const
        {
            LocalCachedFileRcPtr cachedFile = DynamicPtrCast<LocalCachedFile>(untypedCachedFile);
            
            // This should never happen.
            if(!cachedFile)
            {
                std::ostringstream os;
                os << "Cannot build Autodesk-1d Op. Invalid cache type.";
                throw Exception(os.str().c_str());
            }
            
            TransformDirection newDir = CombineTransformDirections(dir,
                fileTransform.getDirection());
            if(newDir == TRANSFORM_DIR_UNKNOWN)
            {
                std::ostringstream os;
                os << "Cannot build file format transform,";
                os << " unspecified transform direction.";
                throw Exception(os.str().c_str());
            }
            
            CreateLut1DOp(ops, cachedFile->lut1D,
                          fileTransform.getInterpolation(), newDir);
        }
    }
    
    FileFormat * CreateFileFormatAutodesk1D()
    {
        return new LocalFileFormat();
    }
}
OCIO_NAMESPACE_EXIT

#ifdef OCIO_UNIT_TEST

namespace OCIO = OCIO_NAMESPACE;
#include "UnitTest.h"

#endif // OCIO_UNIT_TEST

"""
Example for how to generate a csp lut that supports an HDR input domain, and a shaper lut.
-- Jeremy Selan
"""

import sys,os
import PyOpenColorIO as OCIO

print ""
print "PyOCIO:", OCIO.__file__
print "version:",OCIO.version
print ""

config = OCIO.Config.CreateFromEnv()
config.sanityCheck()

INPUT_SPACE = 'scene_grey18'
SHAPER_SPACE = 'log'
OUTPUT_SPACE = 'Dreamcolor'
OUTPUT_FILENAME = os.path.realpath('unclipped.csp')

# Create the mapping from a uniformly sampled ldr output, to a non-uniform hdr input.
shapersize = 2**10
shaperOutData = []
for i in xrange(shapersize):
    x = i/(shapersize-1.0)
    shaperOutData.extend((x,x,x))
shaperToInput = config.getProcessor(SHAPER_SPACE, INPUT_SPACE)
shaperInData = shaperToInput.applyRGB(shaperOutData)

# Create a 3D lut from the 1D shaper space to the 3D output space
lut3dsize = 32
lut3dInputData = []
# Build an identity lut
for i in xrange(lut3dsize):
    for j in xrange(lut3dsize):
        for k in xrange(lut3dsize):
            r = k/(lut3dsize-1.0)
            g = j/(lut3dsize-1.0)
            b = i/(lut3dsize-1.0)
            lut3dInputData.extend((r,g,b))
# map it from shaper space to output space. (This assumes a 1d shaper, for now)
shaperToOutput = config.getProcessor(SHAPER_SPACE, OUTPUT_SPACE)
lut3dOuputData = shaperToOutput.applyRGB(lut3dInputData)

# Write out the file
f = file(OUTPUT_FILENAME,'w')
f.write('CSPLUTV100\n')
f.write('3D\n')
f.write('BEGIN METADATA\n')
f.write('END METADATA\n')
f.write('\n')

# Write 1D
for channel in range(3):
    f.write(str(len(shaperInData)/3) + "\n")
    for i in xrange(len(shaperInData)/3):
        f.write("%f " % shaperInData[3*i+channel])
    f.write("\n")
    for i in xrange(len(shaperOutData)/3):
        f.write("%f " % shaperOutData[3*i+channel])
    f.write("\n")
f.write('\n')

# Write 3D
f.write('%d %d %d\n' % (lut3dsize,lut3dsize,lut3dsize))
for i in xrange(len(lut3dOuputData)/3):
    f.write("%f %f %f\n" % (lut3dOuputData[3*i+0], lut3dOuputData[3*i+1], lut3dOuputData[3*i+2]))
f.write('\n')

f.close()
print 'Wrote',OUTPUT_FILENAME

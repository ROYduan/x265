#!/bin/sh

mkdir -p 8bit 10bit 12bit

cd 12bit
cmake ../../../source -DHIGH_BIT_DEPTH=ON -DEXPORT_C_API=OFF -DENABLE_SHARED=OFF -DENABLE_CLI=OFF -DMAIN12=ON
make ${MAKEFLAGS}

cd ../10bit
cmake ../../../source -DHIGH_BIT_DEPTH=ON -DEXPORT_C_API=OFF -DENABLE_SHARED=OFF -DENABLE_CLI=OFF
make ${MAKEFLAGS}

cd ../8bit
ln -sf ../10bit/libs265.a libs265_main10.a
ln -sf ../12bit/libs265.a libs265_main12.a
cmake ../../../source -DEXTRA_LIB="s265_main10.a;s265_main12.a" -DEXTRA_LINK_FLAGS=-L. -DLINKED_10BIT=ON -DLINKED_12BIT=ON
make ${MAKEFLAGS}

# rename the 8bit library, then combine all three into libs265.a
mv libs265.a libs265_main.a

uname=`uname`
if [ "$uname" = "Linux" ]
then

# On Linux, we use GNU ar to combine the static libraries together
ar -M <<EOF
CREATE libs265.a
ADDLIB libs265_main.a
ADDLIB libs265_main10.a
ADDLIB libs265_main12.a
SAVE
END
EOF

else

# Mac/BSD libtool
libtool -static -o libs265.a libs265_main.a libs265_main10.a libs265_main12.a 2>/dev/null

fi
